
ifeq ($(findstring Windows, $(OS)),Windows)
	DEL := rmdir /S /Q
	MKDIR := mkdir
else
	DEL := rm -rf
	MKDIR := mkdir -p
endif

ifneq (,$(findstring zephyr,$(CC)))
PREFIX ?= .
OBJ_DIR ?= $(PREFIX)/obj
LIB_DIR ?= $(PREFIX)/lib
all:
	-$(MKDIR) "$(OBJ_DIR)"
	-$(MKDIR) "$(LIB_DIR)"
	$(CC) -c $(CFLAGS) -MD -I src src/ld2410_radar.c -o $(OBJ_DIR)/ld2410_radar.o
	$(AR) -rcs $(LIB_DIR)/libld2410_radar.a $(OBJ_DIR)/ld2410_radar.o

clean:
	 $(DEL) "$(OBJ_DIR)" "$(LIB_DIR)"

endif
