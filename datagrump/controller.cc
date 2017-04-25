#include <iostream>
#include <math.h>
#include <vector>
#include <cmath>

#include "controller.hh"
#include "timestamp.hh"

using namespace std;

const unsigned int DEFAULT_WIN = 60;
const uint64_t DELAY_THRESH = 80;
unsigned int the_window_size = DEFAULT_WIN;
uint64_t rtt_min = DELAY_THRESH;
uint64_t last_seq_sent = 0;
uint64_t flight_counter = 0;
float silly_win = DEFAULT_WIN;
const int PREDICTION_SIZE = 2;
uint64_t timestamp_prev_ack_received = 0;
vector<vector<uint64_t>> recent_acks(PREDICTION_SIZE);
int ack_counter = 0;

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
  last_seq_sent = sequence_number;

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
   
  uint64_t rtt = timestamp_ack_received - send_timestamp_acked;

  /* update rtt_min only for actual encountered rtt */
  if (rtt < rtt_min) {
    rtt_min = rtt;
  }

  /* record most recent 2 x,y pair of time for ACK and its rtt */
  vector<uint64_t> datagram_RTT = {timestamp_ack_received, rtt};

  if (ack_counter < PREDICTION_SIZE) {
    recent_acks[ack_counter] = datagram_RTT;
  } else {
    recent_acks[0] = recent_acks[1];    
    recent_acks[1] = datagram_RTT;
    /* ignore acks received at the same time */
    if (float(recent_acks[1][0])-float(recent_acks[0][0]) > 0) {
      float slope = (float(recent_acks[1][1])-float(recent_acks[0][1]))/(float(recent_acks[1][0])-float(recent_acks[0][0]));
     
      uint64_t predicted_time_to_next_ack = timestamp_ack_received - timestamp_prev_ack_received;
      float intercept = float(recent_acks[1][1]) - slope*float(recent_acks[1][0]);
      float new_rtt = ceil(slope*(float(timestamp_ack_received) + float(predicted_time_to_next_ack)) + intercept);
      if(new_rtt > 0) {
        rtt = new_rtt;
      }
    } 
  }

  timestamp_prev_ack_received = timestamp_ack_received;
  ack_counter++;

  /* Keep track of datagrams in flight and only decrease window size once every group of datagrams instead of every ack received */
  uint64_t number_in_flight = last_seq_sent - sequence_number_acked; 
  
  if (rtt > timeout_ms()) {
    if (flight_counter == 0 ) {
      float resize_factor = (rtt-(timeout_ms()/2))/(timeout_ms()/2);
      the_window_size = ceil(the_window_size/(resize_factor));
      silly_win = the_window_size;
      flight_counter = floor(number_in_flight);
    } else { flight_counter--; }
  } else {
    silly_win = silly_win + 1.0/the_window_size;
    the_window_size = floor(silly_win);
    /*if (the_window_size > DEFAULT_WIN) {
      the_window_size = DEFAULT_WIN; 
    }*/
  }

  if ( debug_ ) {
    cerr << "At time " << timestamp_ack_received
	 << " received ack for datagram " << sequence_number_acked
	 << " (send @ time " << send_timestamp_acked
	 << ", received @ time " << recv_timestamp_acked << " by receiver's clock)"
	 << "RTT is " << rtt << " RTTmin is " << rtt_min
	 << endl;
  }
}

/* How long to wait (in milliseconds) if there are no acks
   before sending one more datagram */
unsigned int Controller::timeout_ms( void )
{
  return min(2*rtt_min, DELAY_THRESH);
}
