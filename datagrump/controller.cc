#include <iostream>
#include <math.h>

#include "controller.hh"
#include "timestamp.hh"

using namespace std;

const unsigned int DEFAULT_WIN = 32;
const uint64_t DELAY_THRESH = 80;
const float WAIT_FACTOR = 2;
unsigned int the_window_size = DEFAULT_WIN;
uint64_t time_since_last_resize = 0;

/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug )
{}

/* Get current window size, in datagrams */
unsigned int Controller::window_size( void )
{
  if ( debug_ ) {
    cerr << "At time " << timestamp_ms()
	 << " window size is " << the_window_size << endl;
  }

  return the_window_size;
}

/* A datagram was sent */
void Controller::datagram_was_sent( const uint64_t sequence_number,
				    /* of the sent datagram */
				    const uint64_t send_timestamp )
                                    /* in milliseconds */
{
  /* Default: take no action */

  if ( debug_ ) {
    cerr << "At time " << send_timestamp
	 << " sent datagram " << sequence_number << endl;
  }
}

/* An ack was received */
void Controller::ack_received( const uint64_t sequence_number_acked,
			       /* what sequence number was acknowledged */
			       const uint64_t send_timestamp_acked,
			       /* when the acknowledged datagram was sent (sender's clock) */
			       const uint64_t recv_timestamp_acked,
			       /* when the acknowledged datagram was received (receiver's clock)*/
			       const uint64_t timestamp_ack_received )
                               /* when the ack was received (by sender) */
{
  /* Keep track of time stamp and only decrease window size every period of time instead of every ack received */

  
  uint64_t rtt = timestamp_ack_received - send_timestamp_acked;

  if (rtt > DELAY_THRESH) {
    if (timestamp_ms() - time_since_last_resize > WAIT_FACTOR*DELAY_THRESH) {
      the_window_size = ceil(the_window_size/2);
      time_since_last_resize = timestamp_ms();
    }
  } else {
    time_since_last_resize = 0;
    the_window_size += 1;
    if (the_window_size > DEFAULT_WIN) {
      the_window_size = DEFAULT_WIN;
    }
  }

  if ( debug_ ) {
    cerr << "At time " << timestamp_ack_received
	 << " received ack for datagram " << sequence_number_acked
	 << " (send @ time " << send_timestamp_acked
	 << ", received @ time " << recv_timestamp_acked << " by receiver's clock)"
	 << "RTT is " << rtt << " time " << time_since_last_resize
	 << endl;
  }
}

/* How long to wait (in milliseconds) if there are no acks
   before sending one more datagram */
unsigned int Controller::timeout_ms( void )
{
  return DELAY_THRESH; /* timeout of one second */
}
