PROGNAME := LogMonitor
SRCDIR := src
SUBDIR := src/lib

SRCS := $(SRCDIR)/LogMonitorApp.cpp \
$(SRCDIR)/LogMonitor.cpp \
$(SUBDIR)/ProcessFramework.cpp 

CXX := g++
CPPFLAGS += 
DEFS :=
LDFLAGS += 
LIBS +=
LDADD := -lpthread -lrt

CXXFLAGS := -Wall -O3

OUTDIR := Build

PROG := $(OUTDIR)/$(PROGNAME)
OBJS := $(SRCS:%.cpp=$(OUTDIR)/%.o)
DEPS := $(SRCS:%.cpp=$(OUTDIR)/%.d)

.PHONY: install clean distclean

all: $(PROG)

-include $(DEPS)

$(PROG): $(OBJS)
	$(CXX) $(LDFLAGS) -o $@ $^ $(LIBS) $(LDADD)

$(OUTDIR)/%.o:%.cpp
	@if [ ! -e `dirname $@` ]; then mkdir -p `dirname $@`; fi
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $(DEFS) -o $@ -c -MMD -MP -MF $(@:%.o=%.d) $<

clean:
	rm -rf $(OUTDIR)

install:
	cp -p $(PROG) ../../
