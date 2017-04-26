#include <iostream>
#include <cmath>

#include "controller.hh"
#include "timestamp.hh"

using namespace std;


const unsigned int DEFAULT_WIN = 50;

/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug ), window_size_( DEFAULT_WIN ), silly_window_ ( DEFAULT_WIN),
    last_sent_seq_( 0 ), last_failed_seq_( 0 )
{}

/* Get current window size, in datagrams */
unsigned int Controller::window_size( void )
{
  if ( debug_ ) {
    cerr << "At time " << timestamp_ms()
	 << " window size is " << window_size_ << endl;
  }

  return window_size_;
}

/* A datagram was sent */
void Controller::datagram_was_sent( const uint64_t sequence_number,
				    /* of the sent datagram */
				    const uint64_t send_timestamp )
                                    /* in milliseconds */
{
  last_sent_seq_ = sequence_number;

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

  uint64_t sample_RTT = timestamp_ack_received - send_timestamp_acked;
  update_window(sample_RTT, sequence_number_acked);

  if ( debug_ ) {
    cerr << "At time " << timestamp_ack_received
	 << " received ack for datagram " << sequence_number_acked
	 << " (send @ time " << send_timestamp_acked
	 << ", received @ time " << recv_timestamp_acked << " by receiver's clock)"
	 << endl;
  }
}

/* How long to wait (in milliseconds) if there are no acks
   before sending one more datagram */
unsigned int Controller::timeout_ms( void )
{
  return 1000; /* timeout of one second */
}

/* Update window size based on new RTT sample */
void Controller::update_window( uint64_t sample_RTT, uint64_t sequence_number_acked ) {
  if( sample_RTT > timeout_ms() && sequence_number_acked > last_failed_seq_) {
    window_size_ = floor(window_size_/2);
    silly_window_ = window_size_;
    last_failed_seq_ = last_sent_seq_;
  }
  else {
    silly_window_ = silly_window_ + 1.0/window_size_;
    window_size_ = floor(silly_window_);
  }
}