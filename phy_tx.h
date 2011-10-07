/*! \file phy_tx.h
 *  Framework for implementing a general packet transmitter. Writes output to a
 *  gr_message object and inserts it into an output queue.
 * 
 * Copyright (C) 2009  The University of Texas at Austin.
 * 
 * This file is part of Hydra: A wireless multihop testbed.
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#ifndef INCLUDED_PHY_TX_H
#define INCLUDED_PHY_TX_H

#include <gr_msg_queue.h>
#include <omnithread.h>
#include <itpp/itbase.h>
#include <itpp/itcomm.h>

class phy_tx
{
  private:
    static int COUNT;

  protected:
    std::string d_name;
    int d_unique_id;
    inline void set_name(const std::string s) { d_name = s; }

  public:
    phy_tx(const unsigned int Ntx=1, gr_msg_queue_sptr outQ=gr_msg_queue_sptr() );
    virtual ~phy_tx();

    
    // MUTATORS
    inline void set_debug(const int d) { d_debug = d; }
    virtual inline void set_ntx(unsigned int n) { d_ntx = n; }
    inline void set_outputQ(const gr_msg_queue_sptr q) { d_outputQ = q; }
    
    inline void set_gain(float g) { d_gain = g; }
    inline void set_gain_db(float gdb) { d_gain = pow(10.0, gdb/10.0); }
    void set_roll_off(const double);
    void set_filter_length(const int);
    void set_upsampling_factor(const int);
    
    // ACCESSORS
    inline int debug() { return d_debug;}
    inline unsigned int ntx() const { return d_ntx; }
    inline gr_msg_queue_sptr outputQ() const { return d_outputQ; }
    inline const std::string name() { return d_name; }
    inline const int unique_id() { return d_unique_id; }
    
    inline const float gain() { return d_gain; }
    inline const float gain_db() { return 10.0*log10(fabs(d_gain) ); }
    inline double roll_off() const { return d_roll_off; }
    inline int filter_length() const { return d_filter_length; }
    inline int upsampling_factor() const { return d_upsampling_factor; }
    
    inline void transmit(gr_message_sptr m) { transmit((char *)m->msg(), m->length() ); }
    inline void transmit(const std::string & data) { transmit(data.data(), data.length() ); }
    virtual void transmit(const char *buf, const int len);
    virtual bool work(const itpp::bvec & payload, itpp::cmat & waveform);

    // HELPER FUNCTIONS
    static const bool MSB_FIRST;
    static const unsigned int BITSPERBYTE;
    static void convert_bytestobits (const char *, const int, itpp::bvec &, bool=phy_tx::MSB_FIRST);
    static void copy_waveform (const itpp::cmat & in, gr_complex* out);

  private:
    int d_debug;  // debug level
    unsigned int d_ntx;
    gr_msg_queue_sptr d_outputQ;
    float d_gain;
    omni_mutex d_mutex;

    // pulse shaper & relevant parameters
    double d_roll_off;
    int d_filter_length;
    int d_upsampling_factor;
    itpp::Raised_Cosine<std::complex<double> > d_shaper;
};

#endif /* INCLUDED_PHY_TX_H */
