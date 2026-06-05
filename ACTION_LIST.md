# MCP251xFD Driver — Action List

Status snapshot and prioritised backlog for the `lw_mcp251xfd` driver.
Paths are relative to the repo root. Supersedes the stale `TODO.md` (which is
git-ignored and describes work that is already done).

## Current state (what already works)

The driver is far more complete than `TODO.md` or `README.md` suggest. Implemented
and exercised by `tests/loopback/loopback_test.c`:

- Instance lifecycle (`create`/`destroy`), reset, deinit (sleep).
- SPI register/word read & write; software reset.
- Operating-mode request/await/change/get.
- Oscillator bring-up (4 MHz + PLL, 20 MHz, 40 MHz) and automatic NBT/DBT bit-timing.
- FIFO configuration with RAM budgeting; TX queue fallback when no FIFOs given.
- Acceptance filters (configure/disable).
- Interrupt enable/get/clear.
- TX (`transmit`, `abort_tx`) and RX (`get_received`, `get_all_received`,
  `rx_pending`, `flush_rx`, `get_rx_overflow`).
- Error state / bus-off recovery, bus diagnostics (BDIAG0/1).
- GPIO control, TEF, wake-up filter, time-base read.

The loopback test passes for classic + FD frames, standard + extended IDs, bursts.

---

## 1. Bugs / correctness (do first)

- [x] **~~Dead CRC SPI path.~~ DONE.** The CRC read helper had two protocol bugs
  (a spurious host command-CRC on reads that shifted the data buffer by 2 bytes, and a
  length field expressed in words instead of bytes for SFR access). Both fixed. CRC is
  now wired through `mcp251xfd_{read,write}_register` behind `config->use_crc`, enabled
  only after oscillator bring-up, with retry (`MCP251XFD_CRC_RETRIES`) and a sticky
  error flag (`mcp251xfd_get_spi_crc_error`). The `_word_crc` wrappers were removed.
  Remaining: `MCP251XFD_INT_SPI_CRC` handling, and consider `WRITE_SAFE` for writes.
- [ ] **`get_model` docstring is wrong.** Header (`mcp251xfd.h:709-719`) says it
  "Reads the DEVID register and identifies…", but the implementation
  (`mcp251xfd.c:1506`) just returns the stored `dev->model`. Either implement real
  detection (the OSC `LPMEN` bit, not DEVID, distinguishes 2517 vs 2518) or fix the
  docstring to say it returns the configured model.
- [ ] **Extended-ID alignment doc contradicts code.** `configure_filter` docstring
  (`mcp251xfd.h:313-314`) says extended IDs must be "left-aligned in the 32-bit
  value", but `mcp251xfd_pack_id` (`mcp251xfd.c:887`) and `transmit` treat `id` as a
  flat **right-aligned** 29-bit value (the loopback test relies on this). Fix the
  docstring to say right-aligned/flat; same wording bug applies to the `mask` param.
- [ ] **Time-base counter is never enabled.** `get_timestamp` (`mcp251xfd.c:1591`)
  reads `CiTBC` and TEF timestamps read the TBC, but nothing ever sets
  `CiTSCON.TBCEN`/prescaler. As shipped, every timestamp reads 0. Add an enable path
  (see §2) and note the dependency in TEF docs.
- [ ] **Default (no-FIFO) config cannot transmit.** When `fifo_count == 0`,
  `initialise` (`mcp251xfd.c:1134-1155`) enables the TX **queue** (CiTXQCON @0x0050)
  and makes FIFO 1 an **RX** FIFO. But `transmit()` only drives CiFIFOCON/STA/UA
  (FIFOs 1–31) and has no path to the TXQ — so out of the box there is no usable TX
  object. **Fix:** either make `transmit(fifo_num==0)` target the TXQ, or change the
  default to a TX FIFO + RX FIFO pair, and document it.
- [ ] **`initialised` flag / `NOT_INITIALISED` are inert.** The flag is set but never
  checked, and `MCP251XFD_RETURN_NOT_INITIALISED` is never returned. Add a
  `CHECK_DEV_INITIALISED(dev)` guard to the runtime entry points (transmit, receive,
  filters, etc.) so misuse fails loudly instead of issuing garbage SPI.
- [ ] **`fifo_configs` lifetime / `reset()` hazard.** `dev->config = *config` is a
  shallow copy; `config->fifo_configs` is retained by pointer. `mcp251xfd_reset()`
  re-runs `initialise(&dev->config)` and dereferences it again — a dangling pointer if
  the caller's array didn't outlive the device. Either deep-copy the FIFO array into
  the instance or document the lifetime requirement loudly.
- [ ] **`fosc` not validated against supported set.** `configure_osc`
  (`mcp251xfd.c:657`) treats anything other than 4 MHz as a direct clock, so an
  unsupported value (e.g. 8/16 MHz) silently produces wrong bit timing. Reject
  `fosc` values not in {4, 20, 40} MHz with `INVALID_PARAM`.

## 2. Missing features / API gaps

- [ ] **Time-base counter configuration.** Add
  `mcp251xfd_configure_timestamp(dev, enable, prescaler)` writing `CiTSCON`
  (TBCEN + TBCPRE). Prerequisite for both `get_timestamp` and TEF timestamps.
- [ ] **Configurable sample point.** Bit timing is hard-coded to 80%
  (`mcp251xfd.c:718-731`). Many networks (CiA / J1939) want 87.5%. Add an optional
  sample-point field to the config (default 80%) plumbed into
  `mcp251xfd_calculate_bit_timing`.
- [ ] **Target operating mode after init.** `initialise` leaves the device in CONFIG
  mode; every caller must remember to switch to NORMAL. Add an optional
  `config->start_mode` (default NORMAL, or CONFIG to opt out) so the happy path is
  one call.
- [ ] **One-shot / restricted-retransmission control.** Expose `CiCON.RTXAT`
  (and per-FIFO TXAT) for one-shot transmit — useful for time-triggered / safety use.
- [ ] **Transient TX-interrupt helper.** The `configure_fifo` comment
  (`mcp251xfd.c:855-862`) tells callers to enable TX "not full" IE transiently but
  offers no API. Add `mcp251xfd_set_tx_interrupt(dev, fifo, enable)`.
- [ ] **Per-FIFO / global interrupt source decode.** Add helpers to read
  `CiTXIF`/`CiRXIF`/`CiTXATIF`/`CiRXOVIF` so an ISR can find *which* FIFO fired,
  and a `CiVEC`-based "what interrupted" helper for fast dispatch.
- [ ] **Static (malloc-free) instantiation.** `create_instance` uses `malloc`, which
  many embedded projects forbid. Expose either `MCP251XFD_INSTANCE_SIZE` + a
  placement-init, or a `mcp251xfd_init_static(void *storage, size_t)` entry point.

## 3. Performance / efficiency

- [ ] **Batch the TX message-object write.** `transmit` issues separate CS-toggled
  transfers for T0, T1, and payload, plus FIFOSTA/FIFOUA reads and a FIFOCON strobe
  (~6 transactions/frame). Pack header+payload into one `write_register` under a
  single CS assertion.
- [ ] **Batch the RX read.** `read_rx_object` similarly does many small transfers;
  read T0+T1+payload in one transaction.
- [ ] **Cache FIFO UA / config.** RX/TX paths re-read FIFOUA and (in `rx_pending`)
  FIFOCON every call. For fixed FIFOs these can be cached at configure time.

## 4. Usability / API ergonomics

- [ ] **`const`-correct `initialise`.** It copies but never mutates `config`; take
  `const mcp251xfd_config_t *`.
- [ ] **Frame-construction helpers.** Add inline helpers (e.g.
  `can_frame_classic(id, data, len)`, `can_frame_fd(...)`, a length→DLC encoder) so
  users don't hand-pack `flags`/`dlc`. `can.h` only decodes DLC today.
- [ ] **Document the HAL callback contract in one place.** CS is active-low,
  `spi_transfer` must handle all three (tx-only / rx-only / duplex) patterns and be
  blocking, and `iface` is passed verbatim. Today this is only inferable from the
  loopback test.
- [ ] **Thread-safety statement.** Each call is a multi-step CS transaction; using the
  driver from both an ISR and the main loop without external locking will corrupt
  transfers. State this explicitly and/or offer a user lock hook.
- [ ] **Smaller `can_frame_t` option.** The fixed 64-byte `data[]` makes
  `get_all_received` buffers large for classic-CAN-only users; consider a
  compile-time payload cap.
- [ ] **Consistent FIFO indexing.** `transmit`/`receive` take 1–31 while filters take
  0–31 and the code internally does `fifo_num - 1`. Document the convention prominently
  (and reconsider whether the off-by-one should be hidden).

## 5. Build / tooling / hygiene

- [ ] **`mcp251xfd_conf.h` workflow.** It's described as a template to copy, yet it's
  the file the library includes directly. Clarify: ship a `*.template` and let users
  provide their own via include path, or document overriding the macros from CMake
  (note: the `MCP251XFD_ENABLE_ERROR_MESSAGES` define was just removed from
  `CMakeLists.txt` — decide where that config now lives).
- [ ] **Missing trailing newline** in `mcp251xfd_conf.h` (see working diff).
- [ ] **No automated/host tests.** `tests/` are on-target Pico binaries only. Add a
  host build with a mock SPI back-end (record/replay register transactions) so CI can
  run `transmit`/`receive`/`pack_id`/bit-timing logic without hardware.
- [ ] **CI.** No GitHub Actions. Add a workflow: build the library, run host tests,
  `-Wall -Wextra -Werror` (which will surface the dead-CRC warnings above).
- [ ] **Compiler warnings clean.** Build once with `-Wall -Wextra` and clear results.
- [ ] **Replace stale `TODO.md`** with this file (or delete it — it's git-ignored and
  obsolete).

---

## Documentation TODO

- [ ] **Rewrite `README.md`.** It currently says only "not in a functional or
  buildable state" — false. It should cover:
  - One-paragraph overview + supported parts (MCP2517FD / MCP2518FD), CAN 2.0 + FD.
  - Feature list and current limitations.
  - Wiring / SPI requirements and supported clock crystals (4/20/40 MHz).
  - **Quick-start**: HAL callbacks, config struct, init → set NORMAL → transmit/receive.
  - HAL callback contract (CS active-low, duplex/half SPI, blocking, `iface`).
  - Integration: CMake `add_subdirectory` / `FetchContent` (the alias target
    `lw_mcp251xfd::lw_mcp251xfd` already exists).
  - Message-RAM budget (2 KB) and the FIFO sizing formula
    (`depth × (8 + payload)`), referencing `get_fifo_ram_usage`.
  - Operating-mode flow (init leaves the device in CONFIG).
  - License pointer.
- [ ] **Populate `examples/`** (currently empty): a minimal polled TX/RX example and
  an interrupt-driven RX example; reference the loopback test as a self-test.
- [ ] **Fix in-code doc bugs:** `get_model` (§1), extended-ID alignment (§1), and add
  a note to `get_timestamp`/TEF that the TBC must be enabled first (§2).
- [ ] **Document the FIFO/filter numbering convention** (1-based FIFOs, 0-based
  filters) once, prominently.
- [ ] **Add a Doxyfile.** The headers are already thoroughly Doxygen-commented; ship a
  config and a `docs` target / published HTML.
- [ ] **CHANGELOG + versioning.** `CMakeLists.txt` declares `VERSION 1.0.0`; add a
  CHANGELOG and a tagging/release note (the README's "not functional" line implies
  pre-1.0 reality).
- [ ] **Register/datasheet cross-reference.** A short table mapping driver functions to
  the datasheet registers they touch would help future maintenance and review.
