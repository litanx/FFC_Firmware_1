#https://www.cs.colby.edu/maxwell/courses/tutorials/maketutor/
#https://www.gnu.org/software/make/manual/make.html
CC=gcc
ODIR = bin
CFLAGS=-I..

SUBDIR=bin

RMODELC=../Core/Src/RefModel.c
RMODELH=../Core/Inc/RefModel.h

BINARY=a

all: $(BINARY)

$(BINARY): $(SUBDIR)/test.o $(SUBDIR)/RefModel.o
	$(CC) $^ -o $@  

$(SUBDIR)/test.o:test.c $(RMODELH) | $(SUBDIR)
	$(CC) $< $(CFLAGS) -g -c -o $@ 

$(SUBDIR)/RefModel.o:$(RMODELC) $(RMODELH) | $(SUBDIR)
	$(CC) $<  -I../Core/Inc -g -c -o $@ 
	
$(SUBDIR):
	mkdir $@

clean: 
	del /s /q $(BINARY).exe *.o

.PHONY: all clean
