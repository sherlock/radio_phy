/* -*- phy_tx.i -*- */
/* 
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
 */

%{
#include <phy_tx.h>
%}

%ignore phy_tx::work(const itpp::bvec & payload, itpp::cmat & waveform);
%ignore phy_tx::MSB_FIRST;
%ignore phy_tx::BITSPERBYTE;
%ignore phy_tx::convert_bytestobits (const char *, const int, itpp::bvec &, bool=phy_tx::MSB_FIRST);
%ignore phy_tx::copy_waveform (const itpp::cmat & in, gr_complex* out);
//%rename(tx) phy_tx;

%include <phy_tx.h>

%pythoncode %{
def tx (*args):
    return phy_tx (*args)
phy_tx.__repr__ = lambda self: "<c++ class %s (%d)>"%(self.name(), self.unique_id() )
%}
