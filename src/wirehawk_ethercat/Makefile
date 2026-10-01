# Standalone build of a6_pp.c against the SOEM static library.
# Builds SOEM first if needed; then:  make a6_pp ; sudo ./a6_pp <ifname>
SOEM_DIR ?= $(HOME)/SOEM
SOEM_LIB  = $(SOEM_DIR)/build/libsoem.a
CFLAGS  = -Wall -O2 -I$(SOEM_DIR)/include -I$(SOEM_DIR)/build/include -I$(SOEM_DIR)/osal -I$(SOEM_DIR)/osal/linux -I$(SOEM_DIR)/oshw/linux
LDFLAGS = $(SOEM_LIB) -lpthread -lm

a6_pp: a6_pp.c $(SOEM_LIB)
	gcc $(CFLAGS) -o $@ $< $(LDFLAGS)

a6_pp_rt: a6_pp_rt.c $(SOEM_LIB)
	gcc $(CFLAGS) -o $@ $< $(LDFLAGS)

a6_csp_rt: a6_csp_rt.c $(SOEM_LIB)
	gcc $(CFLAGS) -o $@ $< $(LDFLAGS)

a6_reset: a6_reset.c $(SOEM_LIB)
	gcc $(CFLAGS) -o $@ $< $(LDFLAGS)

a6_two_motors: a6_two_motors.c $(SOEM_LIB)
	gcc $(CFLAGS) -o $@ $< $(LDFLAGS)

a6_gains: a6_gains.c $(SOEM_LIB)
	gcc $(CFLAGS) -o $@ $< $(LDFLAGS)

a6_set_profile: a6_set_profile.c $(SOEM_LIB)
	gcc $(CFLAGS) -o $@ $< $(LDFLAGS)

a6_step_test: a6_step_test.c $(SOEM_LIB)
	gcc $(CFLAGS) -o $@ $< $(LDFLAGS)

# Convenience: clone + build SOEM if it isn't there yet.
$(SOEM_LIB):
	@echo "SOEM library not found. Building it..."
	cd $(SOEM_DIR) && cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build

clean:
	rm -f a6_pp a6_pp_rt a6_csp_rt a6_reset a6_two_motors

.PHONY: clean
