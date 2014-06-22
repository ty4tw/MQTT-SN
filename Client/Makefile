PROGNAME := TomyClient
SRCDIR := src
SUBDIR := src/lib

SRCS := $(SRCDIR)/LinuxClientSample.cpp \
$(SUBDIR)/mqttsnClientAppFw4Linux.cpp \
$(SUBDIR)/mqttsn.cpp \
$(SUBDIR)/mqttsnClient.cpp \
$(SUBDIR)/zbeeStack.cpp \
$(SUBDIR)/udpStack.cpp \
$(SUBDIR)/mqUtil.cpp 

CXX := g++
CPPFLAGS += 
DEFS :=
LDFLAGS += 
LIBS +=

CXXFLAGS := -Wall -O3

OUTDIR := Build

PROG := $(OUTDIR)/$(PROGNAME)
OBJS := $(SRCS:%.cpp=$(OUTDIR)/%.o)
DEPS := $(SRCS:%.cpp=$(OUTDIR)/%.d)

.PHONY: install clean distclean

all: $(PROG)

-include $(DEPS)

$(PROG): $(OBJS)
	$(CXX) $(LDFLAGS) -o $@ $^ $(LIBS)

$(OUTDIR)/%.o:%.cpp
	@if [ ! -e `dirname $@` ]; then mkdir -p `dirname $@`; fi
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $(DEFS) -o $@ -c -MMD -MP -MF $(@:%.o=%.d) $<

clean:
	rm -rf $(OUTDIR)

distclean:
	rm -rf Build
	
install:
	cp -p $(PROG) ../../
	
