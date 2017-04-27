#include <iostream>
#include <cmath>

#include "controller.hh"
#include "timestamp.hh"

using namespace std;


const unsigned int DEFAULT_WIN = 50;
const unsigned int DEFAULT_TO = 1000;
const float DEFAULT_RTT = 100;
const float ALPHA = 0.125;
const float BETA = 0.25;

/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug ), window_size_( DEFAULT_WIN ), silly_window_ ( DEFAULT_WIN ),
    last_sent_seq_( 0 ), flight_counter_( 0 ), timeout_ms_( DEFAULT_TO ),
    estimated_RTT_( DEFAULT_RTT ), variance_RTT_( 0 )
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
  update_RTT(sample_RTT, sequence_number_acked == 0);
  update_timeout();

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
  return timeout_ms_;
}

/* Update window size based on new RTT sample */
void Controller::update_window( uint64_t sample_RTT, uint64_t sequence_number_acked ) {
  if( sample_RTT > timeout_ms() ) {
    if (flight_counter_ == 0) {
      window_size_ = floor(window_size_/2);
      silly_window_ = window_size_;
      flight_counter_ = last_sent_seq_ - sequence_number_acked; 
    } else { flight_counter_--; }    
  }
  else {
    silly_window_ = silly_window_ + 1.0/window_size_;
    window_size_ = floor(silly_window_);
  }
}

/* Update RTT estimate based on latest sample */
void Controller::update_RTT ( uint64_t sample_RTT, bool first ) 
{
  if ( first ) {
    estimated_RTT_ = sample_RTT;
    variance_RTT_ = estimated_RTT_/2;
  }
  else {
    /* update RTT according to RFC 2988 */
    variance_RTT_ = (1 - BETA) * variance_RTT_ + BETA * abs(sample_RTT - estimated_RTT_);
    estimated_RTT_ =  (1 - ALPHA) * estimated_RTT_ + ALPHA * sample_RTT;
  }
}

/* Update retransmission timeout value*/
void Controller::update_timeout( void ) 
{
  timeout_ms_ = ceil(estimated_RTT_ + 4 * variance_RTT_);
}