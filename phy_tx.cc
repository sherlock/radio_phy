/*! \file phy_tx.cc
 *  Implementation of phy_tx.
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

/* Enable assert() statement */
#ifdef NDEBUG
#undef NDEBUG
#endif

#include <phy_tx.h>

// Make sure waveform returned from work() has ntx() rows.
#define CHECK_WAVEFORM_NTX   0

static const float EPSILON = 1.0e-8;

int  phy_tx::COUNT = 0;
const bool phy_tx::MSB_FIRST = true;
const unsigned int phy_tx::BITSPERBYTE = 8;

void
phy_tx::convert_bytestobits (const char *in, const int len, itpp::bvec & out, bool msb)
{
  unsigned int byte=0, mask=0;
  out.set_size(len* phy_tx::BITSPERBYTE);
  for (unsigned int k=0; k<(unsigned int)len; k++) {
    byte = in[k]; mask = (msb)? (1<<(phy_tx::BITSPERBYTE-1) ) : 1;
    for (unsigned int j=0; j<phy_tx::BITSPERBYTE; j++) {
      out[k*phy_tx::BITSPERBYTE+j] = ((byte & mask) == 0) ? 0:1;
      mask = (msb)? (mask>>1):(mask<<1);
    }
  }
}

void
phy_tx::copy_waveform (const itpp::cmat & in, gr_complex* out)
{
  // Take advantage of itpp::cmat::_data() structure.
  // Data is stored as interleaved columns (think of vec(X) operation).
  // Must convert gr_complexd to gr_complex.

  gr_complexd * din = (gr_complexd *) in._data();
  for (int k=0; k<in.size(); k++) out[k] = (gr_complex) din[k];
}

phy_tx::phy_tx(const unsigned int Ntx, gr_msg_queue_sptr outQ)
  : d_name("phy_tx"), d_unique_id(0), d_debug(1), d_ntx(Ntx), d_outputQ(outQ),
    d_gain(1.0), d_roll_off(0.5), d_filter_length(8), d_upsampling_factor(1)
{
  d_unique_id = phy_tx::COUNT;
  phy_tx::COUNT ++;

  if (!outputQ() ) set_outputQ(gr_make_msg_queue() );
  assert(ntx() );
  
  d_shaper = itpp::Raised_Cosine<std::complex<double> > (roll_off(), filter_length(), upsampling_factor() );
}

phy_tx::~phy_tx() {}

/* interpolates waveform using Raised Cosine pulse
 * Because of FIR filtering, output waveform will have additional samples on the
 * leading edge. There number of leading samples will be: upsampling_factor*(filter_length/2).
 * Thus, the total number of samples returned by this method will be:
 * upsampling_factor*(filter_length/2 + input_length)
 */
static void
_interpolate_waveform(const itpp::cmat & waveform, itpp::cmat & out, itpp::Raised_Cosine<std::complex<double> > & shaper)
{
  int padded_size = waveform.cols()+shaper.get_pulse_length()/2+1;
  int output_size = padded_size*shaper.get_upsampling_factor();
  std::vector<itpp::cvec> padded_data(waveform.rows(), itpp::cvec(padded_size) );
  for (int k=0; k<waveform.rows(); k++) {
    //padded_data[k].zeros();
    padded_data[k].set_subvector(0, waveform.get_row(k));
    padded_data[k].set_subvector(waveform.cols(), padded_data[k].size()-1, gr_complexd(0,0) );
  }

  out.set_size(waveform.rows(), output_size);
  shaper.clear();
  for (int k=0; k<waveform.rows(); k++) {
    out.set_row(k, shaper.shape_symbols(padded_data[k]) );
    shaper.clear();
  }
}

void
phy_tx::transmit(const char *buf, const int len)
{
  omni_mutex_lock l(d_mutex);

  itpp::bvec payload;
  itpp::cmat waveform, upsample_waveform;
  gr_message_sptr m = gr_message_sptr();
  bool success = false;

  itpp::cmat *txwaveform = &waveform; // pointer to output waveform
  
  // convert data to bit vector
  phy_tx::convert_bytestobits(buf, len, payload);

  // pass bit vector to work() to get waveform
  success = work (payload, waveform);

  // create gr_message containing txwaveform returned from work()
  if (success) {
    // interpolate if necessary
    if (upsampling_factor() > 1) {
      _interpolate_waveform(waveform, upsample_waveform, d_shaper);
      txwaveform = &upsample_waveform;
    }
    
    // apply gain if necessary
    if ( (gain() > 1.0+EPSILON) || (gain() < 1-EPSILON) )
      (*txwaveform) = (*txwaveform) * gr_complexd(gain()/std::sqrt(float(ntx() ) ),0);
    
#if CHECK_WAVEFORM_NTX
    // make sure returned txwaveform conforms with ntx()
    assert(txwaveform->rows() == ntx() )
#endif
    long wlen = txwaveform->size() * sizeof(gr_complex);
    m = gr_make_message(0, txwaveform->rows(), txwaveform->cols(), wlen);
    phy_tx::copy_waveform(*txwaveform, (gr_complex *)m->msg() );
  }

  // send message to outputQ()
  if (m) outputQ()->insert_tail(m);
  // might want additional functionality (e.g. send TXDONE message)

}

/*
 * payload = bit vector
 * waveform = (ntx() x M ) matrix
 * return bool, true iff. work() was succesful
 */
bool
phy_tx::work(const itpp::bvec & payload, itpp::cmat & waveform)
{
  const gr_complexd BPSK_0 = gr_complexd(-1,0);
  const gr_complexd BPSK_1 = gr_complexd( 1,0);
  waveform.set_size(ntx(), payload.size()/ntx() );
  for (int k=0; k<waveform.cols(); k++)
    for (unsigned int j=0; j<ntx(); j++)
      waveform(k,j) = (payload[k*ntx()+j])? BPSK_1 : BPSK_0;

  return true;
}

void
phy_tx::set_roll_off(const double x)
{
  assert((x>=0.0) && (x<=1.0));
  d_roll_off = x;
  d_shaper = itpp::Raised_Cosine<std::complex<double> > (roll_off(), filter_length(), upsampling_factor() );
}

void
phy_tx::set_filter_length(const int n)
{
  assert(n>0);
  d_filter_length = n;
  d_shaper = itpp::Raised_Cosine<std::complex<double> > (roll_off(), filter_length(), upsampling_factor() );
}

void
phy_tx::set_upsampling_factor(const int m)
{
  assert(m>0);
  d_upsampling_factor = m;
  d_shaper = itpp::Raised_Cosine<std::complex<double> > (roll_off(), filter_length(), upsampling_factor() );
}
