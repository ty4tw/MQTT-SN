PROGNAME := TomyGateway
SRCDIR := src
SUBDIR := src/lib

SRCS := $(SRCDIR)/TomyGateway.cpp \
$(SRCDIR)/BrokerRecvTask.cpp \
$(SRCDIR)/BrokerSendTask.cpp \
$(SRCDIR)/ClientRecvTask.cpp \
$(SRCDIR)/ClientSendTask.cpp \
$(SRCDIR)/GatewayControlTask.cpp \
$(SRCDIR)/GatewayResourcesProvider.cpp \
$(SUBDIR)/sem_timedwait.cpp \
$(SUBDIR)/ProcessFramework.cpp \
$(SUBDIR)/Messages.cpp \
$(SUBDIR)/TCPStack.cpp \
$(SUBDIR)/TLSStack.cpp \
$(SUBDIR)/Topics.cpp \
$(SUBDIR)/UDPStack.cpp \
$(SUBDIR)/XXXXXStack.cpp \
$(SUBDIR)/ZBStack.cpp 

CXX := g++
CPPFLAGS += 
DEFS :=
LDFLAGS += 
LIBS +=
LDADD := -lpthread -lssl -lcrypto

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
