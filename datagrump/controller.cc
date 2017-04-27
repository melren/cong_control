#include <iostream> 
#include <math.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <numeric>

#include "controller.hh"
#include "timestamp.hh"

using namespace std;

const unsigned int DEFAULT_WIN = 32;		// default cwnd size
const uint64_t MAX_DELAY = 80;		// maximum desired latency
const int PREDICTION_SIZE = 4;			// number of recent ACKs to track for prediction of next RTT

/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug ), window_size_( DEFAULT_WIN ), silly_window_( DEFAULT_WIN ),
    rtt_min_( MAX_DELAY ), prev_rtt_( 0 ), last_seq_sent_( 0 ), flight_counter_( 0 ),
    timestamp_prev_ack_received_( 0 ), ack_counter_( 0 ), ACK_times_(PREDICTION_SIZE), 
    ACK_RTTs_(PREDICTION_SIZE)
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
  /* Default: take no action */
  last_seq_sent_ = sequence_number;

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
  timestamp_prev_ack_received_ = timestamp_ack_received; 
  ack_counter_++;
  
  // update rtt_min_ 
  if (rtt < rtt_min_) {
    rtt_min_ = rtt;
  }
  if ( debug_ ) {
    cerr << "At time " << timestamp_ack_received
   << " received ack for datagram " << sequence_number_acked
   << " (send @ time " << send_timestamp_acked
   << ", received @ time " << recv_timestamp_acked << " by receiver's clock)"
   << "RTT is " << rtt << " RTTmin is " << rtt_min_
   << endl;
  }
  
  ACK_times_[ack_counter_ % PREDICTION_SIZE] = timestamp_ack_received;
  ACK_RTTs_[ack_counter_ % PREDICTION_SIZE] = rtt;
  
  bool predicted = false;
  if(ack_counter_ > PREDICTION_SIZE){ 
    /* do not double count acks received at the same time */
    if(timestamp_ack_received == timestamp_prev_ack_received_){
      ack_counter_--;
    } else {
      rtt = predicted_RTT(timestamp_ack_received);
      predicted = true;
    }
  }
  update_window (rtt, sequence_number_acked, predicted);
  prev_rtt_ = rtt;
}

/* Update window size based on new RTT sample */
void Controller::update_window( uint64_t rtt, uint64_t sequence_number_acked, bool predicted) 
{
  if (rtt > timeout_ms()) {
    if ( flight_counter_ == 0 ) { /*  only decrease window size once per group of datagrams */
      float resize_factor = (rtt-(timeout_ms()/2.0))/(timeout_ms()/2.0);
      if (predicted) {
        /* decrease window less aggressively if using a prediction */
        window_size_ = ceil(window_size_/2.0);
      } else {
        window_size_ = ceil(window_size_/(resize_factor));
      }
      silly_window_ = window_size_;
      flight_counter_ = last_seq_sent_ - sequence_number_acked; 
    } else { flight_counter_--; }
  } else {
    float scale_factor = 1.0;
    if (rtt < prev_rtt_) {
      /* increase window more aggressively if RTT is trending downwards */
      scale_factor = 2*((prev_rtt_ - rtt)/float(prev_rtt_) + 1);
    } 
    silly_window_ = silly_window_ + (scale_factor*1.5)/(float(window_size_));
    window_size_ = floor(silly_window_);
  }
}

/* Fit regression line of most recent ACK time+rtt pairs to predict next RTT 
   credit for the finding the slope of the regression goes to the following post: 
   http://stackoverflow.com/a/19039500 */
uint64_t Controller::predicted_RTT(uint64_t time)
{
  int n = ACK_times_.size();
  double s_x = accumulate(ACK_times_.begin(), ACK_times_.end(), 0.0);
  double s_y = accumulate(ACK_RTTs_.begin(), ACK_RTTs_.end(), 0.0);
  double s_xx = inner_product(ACK_times_.begin(), ACK_times_.end(), ACK_times_.begin(), 0.0);
  double s_xy =  inner_product(ACK_times_.begin(), ACK_times_.end(), ACK_RTTs_.begin(), 0.0);
  double slope = (n*s_xy - s_x*s_y)/(n*s_xx - s_x*s_x);

  double intercept = double(s_y/n) - slope*double(s_x/n);
  uint64_t predicted_time_to_next_ack = time - timestamp_prev_ack_received_;
  
  double pred_rtt = ceil(slope*(double(time) + double(predicted_time_to_next_ack)) + intercept);
  
  /* we don't want to predict something more optimistic than what's possible */
  if(pred_rtt >= rtt_min_) {
    return uint64_t(pred_rtt);
  } else {
    return rtt_min_;
  }
}

/* How long to wait (in milliseconds) if there are no acks
   before sending one more datagram */
unsigned int Controller::timeout_ms( void )
{
  return min(2*rtt_min_, MAX_DELAY);
}
