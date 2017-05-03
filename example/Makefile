CC = gcc 
CXX = g++

CPPFLAGS = -I..
CFLAGS = -g -std=gnu99 -Wall -Wno-unused-parameter -Wno-unused-function -O4
CXXFLAGS = -std=c++11 -g -Wall -O4
LDFLAGS = -lpthread -lm -lcurl 

TARGETS := apriltag_demo opencv_demo

.PHONY: all
all: apriltag_demo

apriltag_demo: apriltag_demo.o ../libapriltag.a
	@echo "   [$@]"
	@$(CC) -o $@ $^ $(LDFLAGS)

opencv_demo: opencv_demo.o Communication.o Fotokite.o FotokiteState.o Serial.o SerialCommunication.o SocketCommunication.o ../libapriltag.a
	@echo "   [$@]"
	@$(CXX) -o $@ $^ $(LDFLAGS) `pkg-config --libs opencv`

%.o: %.c
	@echo "   $@"
	@$(CC) -o $@ -c $< $(CFLAGS) $(CPPFLAGS)

%.o: %.cc %.cpp
	@echo "   $@"
	@$(CXX) -o $@ -c $< $(CXXFLAGS) $(CPPFLAGS)

.PHONY: clean
clean:
	@rm -rf *.o $(TARGETS)
