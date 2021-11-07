
DRV_NAME=i2c_drv

SYS_INC=$(TOP)/incl
SYS_LIB=$(OBJ)/lib

all: $(SYS_INC) $(SYS_INC)/$(DRV_NAME).h $(SYS_LIB) $(SYS_LIB)/$(DRV_NAME).o

$(SYS_INC):
	mkdir -p $@

$(SYS_LIB):
	mkdir -p $@

$(SYS_INC)/$(DRV_NAME).h: $(DRV_NAME).h
	cp $^ $@

$(SYS_LIB)/$(DRV_NAME).o: $(DRV_NAME).o
	cp $^ $@




drv_clean:
	rm -rf *.o

