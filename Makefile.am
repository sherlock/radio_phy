#
# 
# Copyright (C) 2009  The University of Texas at Austin.
# 
# This file is part of Hydra: A wireless multihop testbed.
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
# 
# 

include $(top_srcdir)/Makefile.common

# Must build subdirectories first
SUBDIRS = 

physrcdir=$(top_srcdir)/src/lib/phy
PHY_INCLUDES = -I$(physrcdir)/common

INCLUDES = $(STD_DEFINES_AND_INCLUDES) $(PHY_INCLUDES) $(ITPPCFLAGS)

noinst_LTLIBRARIES = libphy-tx.la

libphy_tx_la_LDFLAGS = $(NO_UNDEFINED) -module -avoid-version $(ITPPLDFLAGS) 
libphy_tx_la_LIBADD  = 	\
	$(PYTHON_LDFLAGS) 		\
	-lstdc++
libphy_tx_la_SOURCES = 	\
	phy_tx.cc

# Install header files
grinclude_HEADERS =	
hydrainclude_HEADERS = 	\
	phy_tx.h
hydraswiginclude_HEADERS = 		\
	phytx.i								\
	phy_tx.i

phytx.i: phy_tx.i
				 $(hydrainclude_HEADERS) $(libphy_tx_la_SOURCES)
	touch phytx.i
