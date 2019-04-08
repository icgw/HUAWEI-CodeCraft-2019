#
# Guowei Chen <icgw@outlook.com>
#   04/08/2018
################################

JUDGE    = main
CXXFLAGS = -std=c++11 -O2
PROGRAM  = $(JUDGE)
CXX      = g++
RM       = rm -f
SRCS     = ../io.cpp traffic.cpp judge.cpp main.cpp
OBJS     = io.o traffic.o judge.o main.o

PHONY += all
all: $(PROGRAM)

$(PROGRAM) : $(OBJS)
	$(CXX) -o $@ $^

$(OBJS): $(SRCS)
	$(CXX) $(CXXFLAGS) -c $^

PHONY += clean
clean:
	$(RM) $(OBJS)

PHONY += all-clean
all-clean: clean
	$(RM) $(PROGRAM)

.PHONY: $(PHONY)
