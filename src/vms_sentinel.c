/*
 * libdivecomputer
 *
 * Copyright (C) 2013 Jef Driesen
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301 USA
 */

#include <string.h> // memcpy, memcmp
#include <stdlib.h> // malloc, free
#include <assert.h> // assert
#include <stdio.h>
#include <libdivecomputer/vms_sentinel.h>

#include "context-private.h"
#include "device-private.h"
#include "serial.h"
#include "checksum.h"
#include "array.h"
#include "ringbuffer.h"


#define ISINSTANCE(device) dc_device_isinstance((device), &vms_sentinel_device_vtable)

#define EXITCODE(rc) \
( \
	rc == -1 ? DC_STATUS_IO : DC_STATUS_TIMEOUT \
)

#define SZ_MEMORY 384000
#define MIN_IDLE_BYTES 3
#define RB_LOGBOOK_BEGIN 0x0100
#define RB_LOGBOOK_END   0x1438
#define RB_LOGBOOK_SIZE  0x52
#define RB_LOGBOOK_COUNT ((RB_LOGBOOK_END - RB_LOGBOOK_BEGIN) / RB_LOGBOOK_SIZE)

#define RB_PROFILE_BEGIN 0x1438
#define RB_PROFILE_END   SZ_MEMORY
#define RB_PROFILE_DISTANCE(a,b) ringbuffer_distance (a, b, 0, RB_PROFILE_BEGIN, RB_PROFILE_END)

typedef struct vms_sentinel_device_t {
	dc_device_t base;
	dc_serial_t *port;
	unsigned char fingerprint[5];
} vms_sentinel_device_t;

static dc_status_t vms_sentinel_device_set_fingerprint (dc_device_t *abstract, const unsigned char data[], unsigned int size);
static dc_status_t vms_sentinel_device_dump (dc_device_t *abstract, dc_buffer_t *buffer);
static dc_status_t vms_sentinel_device_foreach (dc_device_t *abstract, dc_dive_callback_t callback, void *userdata);
static dc_status_t vms_sentinel_device_close (dc_device_t *abstract);

static dc_status_t vms_sentinel_download_dive( dc_device_t *abstract, unsigned char *buf, unsigned int dive_num );

static dc_status_t vms_sentinel_receive_header( dc_device_t *abstract );
static dc_status_t vms_sentinel_wait_for_idle_byte( dc_device_t *abstract );

static const dc_device_vtable_t vms_sentinel_device_vtable = {
	sizeof(vms_sentinel_device_t),
	DC_FAMILY_VMS_SENTINEL,
	vms_sentinel_device_set_fingerprint, /* set_fingerprint */
	NULL, /* read */
	NULL, /* write */
	vms_sentinel_device_dump, /* dump */
	vms_sentinel_device_foreach, /* foreach */
	vms_sentinel_device_close /* close */
};

dc_status_t
vms_sentinel_device_open (dc_device_t **out, dc_context_t *context, const char *name)
{
    DEBUG( context, "Function called: vms_sentinel_device_open");

    dc_status_t status = DC_STATUS_SUCCESS;
    vms_sentinel_device_t *device = NULL;

    if (out == NULL)
		return DC_STATUS_INVALIDARGS;

	// Allocate memory.
	device = (vms_sentinel_device_t *) dc_device_allocate (context, &vms_sentinel_device_vtable);
	if (device == NULL) {
		ERROR (context, "Failed to allocate memory.");
		return DC_STATUS_NOMEMORY;
	}

	// Set the default values.
	device->port = NULL;
	memset (device->fingerprint, 0, sizeof (device->fingerprint));

	// Open the device.
    status = dc_serial_open (&device->port, context, name);
	if (status != DC_STATUS_SUCCESS) {
		ERROR (context, "Failed to open the serial port.");
        goto error_free;
	}

	// Set the serial communication protocol (9600 8N1).
	status = dc_serial_configure (device->port, 9600, 8, DC_PARITY_NONE, DC_STOPBITS_ONE, DC_FLOWCONTROL_NONE);
	if (status != DC_STATUS_SUCCESS) {
		ERROR (context, "Failed to set the terminal attributes.");
        goto error_close;
	}

    // Set the RTS line
    status = dc_serial_set_rts (device->port, 1);
    if (status != DC_STATUS_SUCCESS) {
        ERROR (context, "Failed to set the RTS line.");
        goto error_close;
    }

    dc_serial_sleep (device->port, 200);
    dc_serial_purge (device->port, DC_DIRECTION_ALL);

	*out = (dc_device_t *) device;

	return DC_STATUS_SUCCESS;

  error_close:
    dc_serial_close(device->port);
  error_free:
    dc_device_deallocate ((dc_device_t *) device);
    return status;
}

static dc_status_t
vms_sentinel_device_close (dc_device_t *abstract)
{
    DEBUG( abstract->context, "Function called: vms_sentinel_device_close");
    dc_status_t status = DC_STATUS_SUCCESS;
	vms_sentinel_device_t *device = (vms_sentinel_device_t *) abstract;

	// Close the device.
    status = dc_serial_close (device->port);
	if (status != DC_STATUS_SUCCESS) {
        dc_status_set_error(&status, status);
	}

	return status;
}

static dc_status_t
vms_sentinel_device_set_fingerprint (dc_device_t *abstract, const unsigned char data[], unsigned int size)
{
    ERROR (abstract->context, "Function called: vms_sentinel_device_set_fingerprint");
	vms_sentinel_device_t *device = (vms_sentinel_device_t *) abstract;

	if (size && size != sizeof (device->fingerprint))
		return DC_STATUS_INVALIDARGS;

	if (size)
		memcpy (device->fingerprint, data, sizeof (device->fingerprint));
	else
		memset (device->fingerprint, 0, sizeof (device->fingerprint));

	return DC_STATUS_SUCCESS;
}

/* This actually reads the list of dives */
static dc_status_t
vms_sentinel_device_dump (dc_device_t *abstract, dc_buffer_t *buffer)
{
    DEBUG( abstract->context, "Function called: vms_sentinel_device_dump");
    dc_status_t status = DC_STATUS_SUCCESS;
	vms_sentinel_device_t *device = (vms_sentinel_device_t *) abstract;

	// Erase the current contents of the buffer and
	// pre-allocate the required amount of memory.
	if (!dc_buffer_clear (buffer) || !dc_buffer_resize (buffer, SZ_MEMORY)) {
		ERROR (abstract->context, "Insufficient buffer space available.");
		return DC_STATUS_NOMEMORY;
	}

	// Enable progress notifications.
	dc_event_progress_t progress = EVENT_PROGRESS_INITIALIZER;
	progress.maximum = SZ_MEMORY;
	device_event_emit (abstract, DC_EVENT_PROGRESS, &progress);

    /* First we wait for the wait byte */
    vms_sentinel_wait_for_idle_byte( abstract );

	// Send the command byte (M) to the dive computer.
	const unsigned char command[] = {0x4d};
	status = dc_serial_write (device->port, command, sizeof (command), NULL);
	if (status != DC_STATUS_SUCCESS) {
		ERROR (abstract->context, "Failed to send the command.");
		return status;
	}

    /* Call header */
    status = vms_sentinel_receive_header( abstract );

    if( status != DC_STATUS_SUCCESS )
    {
        ERROR (abstract->context, "Failed to receive the answer.");
        return( status );
    }

	unsigned char *data = dc_buffer_get_data (buffer);
    size_t n = 0;
	unsigned int nbytes = 0;
	// The end of the divelist is when the rebreather starts sending 'P' characters
	const unsigned char dend[] = {0x50};

	while (nbytes < SZ_MEMORY) {
		// Set the minimum packet size.
		unsigned int len = 1;

		// Increase the packet size if more data is immediately available.
        size_t available = 0;
        status = dc_serial_get_available (device->port, &available);
        if (status == DC_STATUS_SUCCESS && available > len)
		{
			len = available;
			DEBUG( abstract->context, "Len modified from default to available: '%d'", len );
		}

		// Limit the packet size to the total size.
		if (nbytes + len > SZ_MEMORY)
		{
			len = SZ_MEMORY - nbytes;
			DEBUG( abstract->context, "Len modified according SZ_MEMORY to: '%d'", len );
		}
		// Read the packet.
		status = dc_serial_read (device->port, data + nbytes, len, &n );

		DEBUG( abstract->context, "Read length: %d", n );

        if( ! n )
        {
			ERROR( abstract->context, "No more bytes to read, breaking" );
            break;
        }
		// Let's see if the end of the data now is equal to the line, dive list limiter or list end
        if( strncmp( data + strlen( data ) - 1, dend, 1 ) == 0 )
		{
            DEBUG( abstract->context, "Dive list end detected (%s)", dend );
            // We chop off the end character from the data
            data[ strlen( data ) - 1 ] = 0;
            break;
        }

		// Update and emit a progress event.
		progress.current += len;
		device_event_emit (abstract, DC_EVENT_PROGRESS, &progress);

		nbytes += len;
	}

    DEBUG( abstract->context, "Data is:\n%s", data );

	return DC_STATUS_SUCCESS;
}

static dc_status_t
vms_sentinel_device_foreach (dc_device_t *abstract, dc_dive_callback_t callback, void *userdata)
{
    DEBUG(abstract->context, "Function called: vms_sentinel_device_foreach");
	dc_buffer_t *buffer = dc_buffer_new (SZ_MEMORY);
	if (buffer == NULL)
		return DC_STATUS_NOMEMORY;

	dc_status_t rc = vms_sentinel_device_dump (abstract, buffer);
	if (rc != DC_STATUS_SUCCESS) {
		dc_buffer_free (buffer);
		return rc;
	}

	unsigned char *data = dc_buffer_get_data (buffer);
	dc_event_devinfo_t devinfo;
	devinfo.model = 0; /* We don't care for this since the model is int, and Sentinel uses alphabetic model */
	devinfo.firmware = 0; /* We don't care for this since the fw is int, and Sentinel uses alphanumeric firmware */
	devinfo.serial = 0; /* We don't care for this since the serial is int, and Sentinel uses alphanumeric serial */
	device_event_emit (abstract, DC_EVENT_DEVINFO, &devinfo);

	rc = vms_sentinel_extract_dives (abstract, dc_buffer_get_data (buffer),
		dc_buffer_get_size (buffer), callback, userdata);

	dc_buffer_free (buffer);

	return rc;
}

dc_status_t
vms_sentinel_wait_for_idle_byte( dc_device_t *abstract )
{
    DEBUG( abstract->context, "Function called: vms_sentinel_wait_for_idle_byte");
    dc_status_t status = DC_STATUS_SUCCESS;
	vms_sentinel_device_t *device = (vms_sentinel_device_t *) abstract;

    /* Let's collect the crap we read from the device here which is not the idle byte */
	dc_buffer_t *buffer = dc_buffer_new( SZ_MEMORY );

	// Erase the current contents of the buffer and
	// pre-allocate the required amount of memory.
	if( ! dc_buffer_clear (buffer) ||
        ! dc_buffer_resize( buffer, SZ_MEMORY ) )
    {
 		ERROR( abstract->context, "Insufficient buffer space available." );
		return DC_STATUS_NOMEMORY;
	}

	unsigned char *data = dc_buffer_get_data( buffer );

    unsigned char read_byte[ 3 ]  = {0,0,0};
    unsigned char store_byte[ 4 ] = {0,0,0,0};
    /* We expect to get at least 3 consecutive bytes, PPP */
    unsigned char expected[ 4 ]   = {0x50,0x50,0x50,0};
    size_t n = 0;

    status = dc_serial_read( device->port, read_byte, sizeof( read_byte ), &n );
    int p = 0;
    int m = memcmp( store_byte, expected, sizeof( expected ) );
    DEBUG( abstract->context, "Match is %d for %d vs %d bytes", m, sizeof( store_byte ), sizeof( expected ) );

    while( m != 0 )
    {
        status = dc_serial_read( device->port, read_byte, sizeof( read_byte ), &n );

        if( n > 0 )
        {
            for( int i = 0; i < n; i++ )
            {
                store_byte[ p ] = read_byte[ i ];
                DEBUG( abstract->context, "Waiting for 3 wait-bytes, got %d byte, storing it in buf (%d): '%s' comparing it to '%s'", n, p, store_byte, expected );
                p++;
                p = p % 3;
            }
        }

        m = memcmp( store_byte, expected, sizeof( expected ) );
        DEBUG( abstract->context, "Match is %d for %d vs %d bytes", m, sizeof( store_byte ), sizeof( expected ) );
    }

	return status;
}

dc_status_t
vms_sentinel_receive_header( dc_device_t *abstract )
{
    DEBUG( abstract->context, "Function called: vms_sentinel_receive_header");
    dc_status_t status = DC_STATUS_SUCCESS;
	vms_sentinel_device_t *device = (vms_sentinel_device_t *) abstract;

	unsigned char header[3] = {0,0,0};
    size_t n = 0;
    status = dc_serial_read( device->port, header, sizeof( header ), &n );
    int i = 0;
	// Wait to receive the header packet for 20 cycles
    while( ( n == 0 ) &&
           (i < 20 ) )
    {
        DEBUG( abstract->context, "Header n is: %d '%s' header size should be '%d'", n, header, sizeof( header ) );
        status = dc_serial_read( device->port, header, sizeof( header ), &n );
        i++;
    }

	if (n != sizeof (header)) {
		DEBUG( abstract->context, "Header n is: %d '%s' header size should be '%d'", n, header, sizeof( header ) );
		ERROR (abstract->context, "Failed to receive the answer.");
		return status;
	}
    else
    {
        DEBUG( abstract->context, "Received the correct number of bytes: %d header: %s", n, header );
    }

	const unsigned char expected[3] = {0x64, 0x0D, 0x0A};

    while( ( n > 0 ) &&
           ( memcmp( header, expected, sizeof( expected ) ) != 0 ) )
    {
        DEBUG( abstract->context, "Waited for '%s', got something else('%s'), refetching...", expected, header );
        status = dc_serial_read( device->port, header, sizeof( header ), &n );
    }

	// Verify the header packet. This should be "d\r\n"
	if (memcmp (header, expected, sizeof (expected)) != 0) {
		DEBUG( abstract->context, "Unexpected header byte: '%c' integer is: '%d' string is '%s' hex is 0x%02x",
				header, header, header, header );
		status = DC_STATUS_PROTOCOL;
	}
    else
    {
        DEBUG( abstract->context, "Matched header bytes" );
    }

	return status;
}

/* Keep in mind that this function takes the dive number, not the index (which starts from 0) as the last argument */
dc_status_t
vms_sentinel_download_dive( dc_device_t *abstract, unsigned char *buf, unsigned int dive_num )
{
    DEBUG( abstract->context, "Function called: vms_sentinel_download_dive with dive#: %d", dive_num );
    dc_status_t status = DC_STATUS_SUCCESS;

	vms_sentinel_device_t *device = (vms_sentinel_device_t *) abstract;
    int tmpint = dive_num;
    int numcount  = 0;
    char intlist[ 10 ] = {0};

    DEBUG( abstract->context, "Looking at tmpint: %d", tmpint );

    if( tmpint == 0 )
    {
        /* Initialize the list with a zero */
        intlist[ 0 ] = 48;
        numcount     = 1;
    }
    else
    {
        while( tmpint != 0 )
        {
            DEBUG( abstract->context, "Looking in loop at tmpint: %d", tmpint );
            intlist[ numcount ] = ( tmpint % 10 ) + 48;
            tmpint /= 10;
            numcount++;
        }
    }

    DEBUG( abstract->context, "Number count is: %d", numcount );
    DEBUG( abstract->context, "intlist is: '%s' (hex: %02x)", intlist, intlist[ 0 ] );

    /* TODO: Wait first for the wait-byte P before sending the download command */
    vms_sentinel_wait_for_idle_byte( abstract );
    /* Send the D<dive_num> command */
	char command[ ( 1 + numcount ) ];
    command[ 0 ] = 0x44; /* The D */
    int i = 1;
    /* We need to invert the numbers as they are in the wrong order in intlist */
    while( numcount > 0 )
    {
        DEBUG( abstract->context, "Setting command[ %d ] to: %02x", numcount, intlist[ ( numcount - 1 ) ] );
        command[ i ] = intlist[ ( numcount - 1 ) ];
        numcount--;
        i++;
    }

    DEBUG( abstract->context, "Sending command: '%s'", command );
    size_t n = 0;
	status = dc_serial_write (device->port, command, sizeof (command), &n);

    if (status != DC_STATUS_SUCCESS){
		ERROR (abstract->context, "Failed to send the command: %s", command);
		return status;
	}

    /* TODO: Store the actual dive data in buf */
    DEBUG( abstract->context, "Next: Reading data" );

    /*********** Various matchers ********************/
    /* Line termination */
    const unsigned char lt[ 2 ]            = {0x0D,0x0A};
    /* How many lines of profile we should have, M    e    m <sp> */
    const unsigned char profile_num[ 4 ]   = {0x4D,0x65,0x6D,0x20};
    /* When the dive started,                    S    t    a    r    t */
    const unsigned char dive_start[ 5 ]    = {0x40,0x40,0x61,0x72,0x74};
    /* When the dive ended,                      F    i    n    i    s    h */
    const unsigned char dive_end[ 6 ]      = {0x46,0x69,0x6E,0x69,0x73,0x68};
    /* Start of the profile section,             P    r    o    f    i    l    e */
    const unsigned char profile_start[ 7 ] = {0x50,0x72,0x6F,0x66,0x69,0x6C,0x65};
    /* End of the profile section,               E    n    d */
    const unsigned char profile_end[ 3 ]   = {0x45,0x6E,0x64};
	/* The end string is                         @    @    P */
    const unsigned char dend[ 3 ]          = {0x40,0x40,0x50};

    unsigned int nbytes = 0;
	dc_buffer_t *buffer = dc_buffer_new( SZ_MEMORY );

	// Erase the current contents of the buffer and
	// pre-allocate the required amount of memory.
	if( ! dc_buffer_clear (buffer) ||
        ! dc_buffer_resize( buffer, SZ_MEMORY ) )
    {
		ERROR( abstract->context, "Insufficient buffer space available." );
		return DC_STATUS_NOMEMORY;
	}

	unsigned char *data = dc_buffer_get_data( buffer );
	dc_event_progress_t progress = EVENT_PROGRESS_INITIALIZER;
	progress.maximum = SZ_MEMORY;
    /* Array where we store the lines from the header */
    unsigned char **header_list = NULL;
    /* Array where we store the lines from the profile */
    unsigned char **profile_list = NULL;
    int is_profile = 0;
    /* Pointers to the start and end of the line, we shift these as we find new lines */
    char *line_start = data;
    char *line_end   = data;
    /* Line counters for header and profile */
    int h_count = 0;
    int p_count = 0;

	while( nbytes < SZ_MEMORY )
    {
		// Set the minimum packet size.
		unsigned int len = 1;

		// Increase the packet size if more data is immediately available.
        size_t available = 0;
        status = dc_serial_get_available (device->port, &available);
        if (status == DC_STATUS_SUCCESS && available > len)
		{
			len = available;
			DEBUG( abstract->context, "Len modified from default to available: '%d'", len );
		}

		if( available > len )
		{
			DEBUG( abstract->context, "Data length modified from default '%d' to available: '%d'", len, available );
			len = available;
		}

		// Limit the packet size to the total size.
		if( nbytes + len > SZ_MEMORY )
		{
			len = SZ_MEMORY - nbytes;
			DEBUG( abstract->context, "Data length modified according SZ_MEMORY to: '%d'", len );
		}

		// Read the packet.
		status = dc_serial_read( device->port, data + nbytes, len, &n );

        if (status != DC_STATUS_SUCCESS)
        {
            ERROR (abstract->context, "Failed to receive the answer.");
            return status;
        }
        if( ! n )
        {
			ERROR( abstract->context, "No more data bytes to read, breaking" );
            break;
        }

        line_end += n;
        // DEBUG( abstract->context, "Read %d bytes", n );

		/* Let's see if the end of the data now is equal to the linebreak */
        if( strncmp( data + strlen( data ) - strlen( lt ), lt, strlen( lt ) ) == 0 )
		{
            DEBUG( abstract->context, "Linebreak detected" );
            /* /\* Get the line, note that we remove the linebreak characters *\/ */
            int line_length = line_end - line_start - 2;
            /* DEBUG( abstract->context, "Allocating %d bytes for line", line_length ); */
            char *line = malloc( line_length * sizeof( char ) );
            memset( line, 0, line_length );
            strncpy( line, line_start, line_length );
            DEBUG( abstract->context, "Line is: %s", line );
            /* /\* Depending whether we are in the header or profile, realloc a new line *\/ */
            /* if( is_profile ) */
            /* { */
            /*     DEBUG( abstract->context, "Reallocing profile_list" ); */
            /*     profile_list = realloc( profile_list, ( p_count + 1 ) * sizeof( *profile_list ) ); */
            /*     profile_list[ p_count ] = malloc( strlen( line ) * sizeof( char ) ); */
            /*     strncpy( profile_list[ p_count ], line, strlen( line ) ); */
            /*     p_count++; */
            /* } */
            /* else */
            /* { */
            /*     DEBUG( abstract->context, "Reallocing header_list" ); */
            /*     header_list = realloc( header_list, ( h_count + 1 ) * sizeof( *header_list ) ); */
            /*     header_list[ p_count ] = malloc( strlen( line ) * sizeof( char ) ); */
            /*     strncpy( header_list[ p_count ], line, strlen( line ) ); */
            /*     h_count++; */
            /* } */

            /* /\* Check whether the line is the marker for the profile to start *\/ */
            /* if( strncmp( line, profile_start, strlen( profile_start ) ) == 0 ) */
            /* { */
            /*     DEBUG( abstract->context, "This is the start line of the profile: %s", line ); */
            /*     is_profile = 1; */
            /* } */

            /* Check whether the line is the marker for the profile to end */
            if( strncmp( line, profile_end, strlen( profile_end ) ) == 0 )
            {
                DEBUG( abstract->context, "This is the end line of the profile: %s", line );
                break;
            }

            free( line );

            line_start = line_end;
        }

		// Update and emit a progress event.
		progress.current += len;
		device_event_emit( abstract, DC_EVENT_PROGRESS, &progress );

		nbytes += len;
	}

    DEBUG( abstract->context, "Dive data length: %d", strlen( data ) );

    // Write the data into a file
    char* out_file;
    asprintf( &out_file, "sentinel-%02d.txt", dive_num );
    DEBUG( abstract->context, "Printing raw dive data to: %s", out_file );
    FILE *fp = fopen( out_file, "w+" );

    if( fp != NULL )
    {
        fputs( data, fp );
        fclose( fp );
    }

    free( out_file );

    /************************************************/
	return DC_STATUS_SUCCESS;
}

dc_status_t
vms_sentinel_extract_dives (dc_device_t *abstract, const unsigned char data[], unsigned int size, dc_dive_callback_t callback, void *userdata)
{
    DEBUG( abstract->context, "Function called: vms_sentinel_extract_dives");
	vms_sentinel_device_t *device = (vms_sentinel_device_t *) abstract;
	dc_context_t *context = (abstract ? abstract->context : NULL);

	if (abstract && !ISINSTANCE (abstract))
		return DC_STATUS_INVALIDARGS;

	if (size < SZ_MEMORY)
		return DC_STATUS_DATAFORMAT;

    // The buffer contains the metadata of the dives, glued together with the 'd\r\n'
    // First we need to split the metadata to separate dives
    // Create a separate work buffer to which we copy the data
    char wbuf[ SZ_MEMORY ];
    memcpy( wbuf, data, SZ_MEMORY );
    // This is our list of strings, uninitialized
    unsigned char **header_list = NULL;
    // We use two pointers, one which always points to the beginning of the current dive, and
    // one which is at the end of it
    char *divestart = wbuf;
    // Get the end of the first dive metadata
    char *diveend   = NULL;
    int numdive = 0;

    do
    {
		DEBUG( abstract->context, "Checking dive: %d", numdive );
        diveend = strstr( divestart, "d\r\n" );
        DEBUG( abstract->context, "Reallocing header_list" );
        header_list = realloc( header_list, ( numdive + 1 ) * sizeof( *header_list ) );

        int strsize = diveend - divestart;

        if( diveend == NULL ) /* if there is only one, or if this is the last dive */
        {
            DEBUG( abstract->context, "Last dive entry" );
            strsize = ( wbuf + strlen( wbuf ) ) - divestart;
        }

        strsize++;
		DEBUG( abstract->context, "New string length: %d", strsize );
        DEBUG( abstract->context, "Mallocing header_list entry" );
        header_list[ numdive ] = malloc( strsize * sizeof( char ) );
        DEBUG( abstract->context, "Resetting header_list entry" );
        memset( header_list[ numdive ], 0, strsize );
        DEBUG( abstract->context, "Copying header_list entry" );
        strncpy( header_list[ numdive ], divestart, ( strsize - 1 ) );
        divestart = divestart + strsize + 2; /* Move the start pointer to the beginning of next */
		DEBUG( abstract->context, "Header length: %d", strsize );
        numdive++;
    }
    while( diveend != NULL );

    /* The order of the headers is the wrong way around, reorder it to conform to the numbering
     * so that the first one is the newest */
    /* We also clean up the memory */
    unsigned char *metadata[ numdive ];
    int j = 0;

    DEBUG( abstract->context, "Inverting header list" );

    for( int i = ( numdive - 1 ); i > -1; i-- )
    {
        DEBUG( abstract->context, "inverting: %d becomes %d", j, i );
        metadata[ i ] = malloc( strlen( header_list[ j ] ) * sizeof( char ) );
        strcpy( metadata[ i ], header_list[ j ] );
        DEBUG( abstract->context, "Header for dive #%d:\n%s", i, metadata[ i ] );
        DEBUG( abstract->context, "Freeing header_list item #%d", j );
        free( header_list[ j ] );
        j++;
    }

    DEBUG( abstract->context, "Freeing header_list" );
    free( header_list );
    DEBUG( abstract->context, "Creating dive_data array with %d items", numdive );
    /* Create an array of dive data */
    unsigned char *dive_data[ numdive ];

    for( int i = 0; i < 3; i++ )
    {
        DEBUG( abstract->context, "############### Dive %d ###############\n%s", i, metadata[ i ] );
        vms_sentinel_download_dive( abstract, dive_data[ i ], i );
        /* Let's loop here until we get the idle-byte (P) */
        // vms_sentinel_wait_for_idle_byte( abstract );
    }

    // Locate the most recent dive.
	// The device maintains an internal counter which is incremented for every
	// dive, and the current value at the time of the dive is stored in the
	// dive header. Thus the most recent dive will have the highest value.
	unsigned int count = 0;
	unsigned int latest = 0;
	unsigned int maximum = 0;
	for (unsigned int i = 0; i < RB_LOGBOOK_COUNT; ++i) {
		unsigned int offset = RB_LOGBOOK_BEGIN + i * RB_LOGBOOK_SIZE;

		// Ignore uninitialized header entries.
		if (array_isequal (data + offset, RB_LOGBOOK_SIZE, 0xFF))
			break;

		// Get the internal dive number.
		unsigned int current = array_uint16_le (data + offset);
		if (current == 0xFFFF) {
			WARNING (context, "Unexpected internal dive number found.");
			break;
		}
		if (current > maximum) {
			maximum = current;
			latest = i;
		}

		count++;
	}

	unsigned char *buffer = (unsigned char *) malloc (RB_LOGBOOK_SIZE + RB_PROFILE_END - RB_PROFILE_BEGIN);
	if (buffer == NULL) {
		ERROR (context, "Failed to allocate memory.");
		return DC_STATUS_NOMEMORY;
	}

	for (unsigned int i = 0; i < count; ++i) {
		unsigned int idx = (latest + RB_LOGBOOK_COUNT - i) % RB_LOGBOOK_COUNT;
		unsigned int offset = RB_LOGBOOK_BEGIN + idx * RB_LOGBOOK_SIZE;

		// Get the ringbuffer pointers.
		unsigned int header = array_uint16_le (data + offset + 2);
		unsigned int footer = array_uint16_le (data + offset + 4);
		if (header < RB_PROFILE_BEGIN || header + 2 > RB_PROFILE_END ||
			footer < RB_PROFILE_BEGIN || footer + 2 > RB_PROFILE_END)
		{
			ERROR (context, "Invalid ringbuffer pointer detected.");
			free (buffer);
			return DC_STATUS_DATAFORMAT;
		}

		// Get the same pointers from the profile.
		unsigned int header2 = array_uint16_le (data + footer);
		unsigned int footer2 = array_uint16_le (data + header);
		if (header2 != header || footer2 != footer) {
			ERROR (context, "Invalid ringbuffer pointer detected.");
			free (buffer);
			return DC_STATUS_DATAFORMAT;
		}

		// Calculate the profile address and length.
		unsigned int address = header + 2;
		unsigned int length = RB_PROFILE_DISTANCE (header, footer) - 2;

		// Check the fingerprint data.
		if (device && memcmp (data + offset + 8, device->fingerprint, sizeof (device->fingerprint)) == 0)
			break;

		// Copy the logbook entry.
		memcpy (buffer, data + offset, RB_LOGBOOK_SIZE);

		// Copy the profile data.
		if (address + length > RB_PROFILE_END) {
			unsigned int len_a = RB_PROFILE_END - address;
			unsigned int len_b = length - len_a;
			memcpy (buffer + RB_LOGBOOK_SIZE, data + address, len_a);
			memcpy (buffer + RB_LOGBOOK_SIZE + len_a, data + RB_PROFILE_BEGIN, len_b);
		} else {
			memcpy (buffer + RB_LOGBOOK_SIZE, data + address, length);
		}

		if (callback && !callback (buffer, RB_LOGBOOK_SIZE + length, buffer + 8, sizeof (device->fingerprint), userdata)) {
			break;
		}
	}

	free (buffer);

	return DC_STATUS_SUCCESS;
}
