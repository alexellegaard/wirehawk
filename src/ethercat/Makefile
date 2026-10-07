# Standalone build of the A6 bench scripts against the SOEM static library.
# Builds SOEM first if needed. Then:  make  (builds all; 'all' is the default)
SOEM_DIR ?= $(HOME)/SOEM
SOEM_LIB  = $(SOEM_DIR)/build/libsoem.a
CFLAGS  = -Wall -O2 -I$(SOEM_DIR)/include -I$(SOEM_DIR)/build/include -I$(SOEM_DIR)/osal -I$(SOEM_DIR)/osal/linux -I$(SOEM_DIR)/oshw/linux
LDFLAGS = $(SOEM_LIB) -lpthread -lm

BINS = a6_pp a6_pp_rt a6_csp_rt a6_reset a6_gains a6_set_profile \
       a6_step_test a6_sync a6_filter a6_gain a6_inertia a6_fault \
       a6_profile_test

all: $(BINS)

a6_pp: a6_pp.c $(SOEM_LIB)
	gcc $(CFLAGS) -o $@ $< $(LDFLAGS)

a6_pp_rt: a6_pp_rt.c $(SOEM_LIB)
	gcc $(CFLAGS) -o $@ $< $(LDFLAGS)

a6_csp_rt: a6_csp_rt.c $(SOEM_LIB)
	gcc $(CFLAGS) -o $@ $< $(LDFLAGS)

a6_reset: a6_reset.c $(SOEM_LIB)
	gcc $(CFLAGS) -o $@ $< $(LDFLAGS)

a6_gains: a6_gains.c $(SOEM_LIB)
	gcc $(CFLAGS) -o $@ $< $(LDFLAGS)

a6_set_profile: a6_set_profile.c $(SOEM_LIB)
	gcc $(CFLAGS) -o $@ $< $(LDFLAGS)

a6_step_test: a6_step_test.c $(SOEM_LIB)
	gcc $(CFLAGS) -o $@ $< $(LDFLAGS)

a6_sync: a6_sync.c $(SOEM_LIB)
	gcc $(CFLAGS) -o $@ $< $(LDFLAGS) -lm

a6_filter: a6_filter.c $(SOEM_LIB)
	gcc $(CFLAGS) -o $@ $< $(LDFLAGS)

a6_gain: a6_gain.c $(SOEM_LIB)
	gcc $(CFLAGS) -o $@ $< $(LDFLAGS)

a6_inertia: a6_inertia.c $(SOEM_LIB)
	gcc $(CFLAGS) -o $@ $< $(LDFLAGS)

a6_fault: a6_fault.c $(SOEM_LIB)
	gcc $(CFLAGS) -o $@ $< $(LDFLAGS)

a6_profile_test: a6_profile_test.c $(SOEM_LIB)
	gcc $(CFLAGS) -o $@ $< $(LDFLAGS)

# Convenience: clone + build SOEM if it isn't there yet.
$(SOEM_LIB):
	@echo "SOEM library not found. Building it..."
	cd $(SOEM_DIR) && cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build

clean:
	rm -f $(BINS)

.PHONY: all clean
