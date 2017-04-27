#ifndef CONTROLLER_HH
#define CONTROLLER_HH

#include <cstdint>
#include <vector>

/* Congestion controller interface */

class Controller
{
private:
  bool debug_; /* Enables debugging output */

  /* Add member variables here */

  /* Involved in window adjustment */
  unsigned int window_size_;
  float silly_window_;
  uint64_t rtt_min_;
  uint64_t prev_rtt_;

  /* Involved in tracking data in flight */
  uint64_t last_seq_sent_;
  uint64_t flight_counter_;
  uint64_t timestamp_prev_ack_received_;
  
  /* Involved in predicting next RTT */
  int ack_counter_;
  std::vector<double> ACK_times_; 
  std::vector<double> ACK_RTTs_;

public:
  /* Public interface for the congestion controller */
  /* You can change these if you prefer, but will need to change
     the call site as well (in sender.cc) */

  /* Default constructor */
  Controller( const bool debug );

  /* Get current window size, in datagrams */
  unsigned int window_size( void );

  /* A datagram was sent */
  void datagram_was_sent( const uint64_t sequence_number,
			  const uint64_t send_timestamp );

  /* An ack was received */
  void ack_received( const uint64_t sequence_number_acked,
		     const uint64_t send_timestamp_acked,
		     const uint64_t recv_timestamp_acked,
		     const uint64_t timestamp_ack_received );

  /* Predict the upcoming RTT based on recent ACKs */
  uint64_t predicted_RTT(uint64_t time);

  /* How long to wait (in milliseconds) if there are no acks
     before sending one more datagram */
  unsigned int timeout_ms( void );
  
  /* Update window size based on new RTT sample */
  void update_window( uint64_t rtt, uint64_t sequence_number_acked, bool predicted);

};

#endif
