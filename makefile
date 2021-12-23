CC = g++ -g -D__LINUX__ -Wall -fPIC
AR = ar -r

PROJ_LIB = libEventArray.a
PROJ_OBJ = EventArray.pb.o

all:$(PROJ_LIB) 

prepare:
    protoc --I=./  --cpp_out=./  EventArray.proto

$(PROJ_LIB):$(PROJ_OBJ)
    $(AR) $(PROJ_LIB) $(PROJ_OBJ)

.cc.o:
    $(CC) -c $< -o $@

clean:
    rm -f *.o
    rm -f EventArray.pb.*