# Raw capture — 2026-08-31

One ski, tapped at the diagnostic port through ae-tracker-db. Bus detected at
**500 kbit/s**; 250k and 125k hear nothing.

A full frame-by-frame CSV was not taken on the day — the tool for it
(`canRawCapture()`, buffers to PSRAM then dumps CSV) was written after the
engine had been shut down and the bus had gone quiet. It now runs automatically
in the `can-sniffer` build, so the next session with the ignition on will
produce `capture-<date>-raw.csv` in this directory. Everything below is what was
actually recorded.

---

## A. Ignition on, engine OFF

`n` is cumulative over two 10 s windows. `var` shows each byte's observed range.

```
0x010 x8 n=910    00 25 A9 0F 00 00 0B 88  var: b6[00-0F] b7[80-8F]
0x012 x8 n=455    40 42 09 00 01 90 0D 97  var: b1[42-43] b4[00-01] b6[00-0F] b7[90-9F]
0x102 x8 n=1000   00 00 1A 4E 5A CC 0D CF  var: b6[00-0F] b7[C0-CF]
0x103 x8 n=1000   00 00 02 00 00 00 0D 0F  var: b6[00-0F] b7[00-0F]
0x122 x8 n=1000   13 17 00 09 14 0D 0F 1B  var: b6[00-0F] b7[10-1F]
0x408 x8 n=451    00 40 4E 4E 00 00 09 49  var: b6[00-0F] b7[40-4F]
0x230 x8 n=899    00 00 00 00 00 00 04 04  var: b6[00-0F] b7[00-0F]
0x410 x8 n=451    03 00 99 99 00 33 06 36  var: b6[00-0F] b7[30-3F]
0x514 x8 n=450    00 89 46 43 92 32 33 FF  var: none (all bytes constant)
0x110 x8 n=500    00 00 00 13 01 01 0F 1C  var: b6[00-0F] b7[10-1F]
0x300 x8 n=500    00 00 16 93 00 66 01 00  var: none (all bytes constant)
0x308 x8 n=500    00 10 00 00 00 00 02 00  var: none (all bytes constant)
0x320 x8 n=500    00 00 00 00 F0 FE 82 00  var: none (all bytes constant)
0x342 x8 n=500    17 E4 99 99 7E 00 16 E0  var: none (all bytes constant)
0x516 x8 n=500    01 2C 89 A0 36 36 4D 42  var: none (all bytes constant)
0x013 x8 n=182    FF 07 09 00 00 00 00 00  var: none (all bytes constant)
```

Seven IDs fully static; every other ID moves only in b6/b7. Nothing is
carrying live data with the engine stopped.

## B. Engine RUNNING (idle, rev to ~6500, back to idle)

Cumulative over five 10 s windows.

```
0x110 x8 n=2502   00 00 00 15 01 01 03 16  var: b3[05-25] b6[00-0F] b7[00-2F]
0x308 x8 n=2502   00 12 00 00 00 00 82 00  var: b0[00-08] b1[10-12] b6[02-82]
0x300 x8 n=2502   00 90 16 93 00 66 00 00  var: b0[00-02] b1[00-EF] b5[18-66] b6[00-01]
0x320 x8 n=2502   00 00 00 00 F0 FE 82 00  var: b4[D0-F0]
0x010 x8 n=4552   00 25 A9 0F 00 00 07 84  var: b6[00-0F] b7[80-8F]
0x342 x8 n=2502   17 E4 99 99 85 00 16 E0  var: b4[62-90]
0x012 x8 n=2276   40 42 09 00 01 90 0B 91  var: b1[42-43] b4[00-5B] b6[00-0F] b7[80-DF]
0x516 x8 n=2502   01 2C 89 A0 36 36 4D 42  var: none (all bytes constant)
0x230 x8 n=4500   00 00 00 00 00 00 02 02  var: b5[00-01] b6[00-0F] b7[00-0F]
0x408 x8 n=2259   00 40 4E 4E 00 00 02 42  var: b6[00-0F] b7[40-4F]
0x410 x8 n=2258   03 00 99 99 00 33 01 31  var: b0[00-03] b5[33-43] b6[00-0F] b7[30-4D]
0x514 x8 n=2258   00 89 46 43 92 32 33 FF  var: none (all bytes constant)
0x102 x8 n=5003   00 00 1B 54 59 CC 06 DC  var: b0[00-74] b1[00-FF] b2[00-84] b3[4D-54] b4[57-59] b6[00-0F] b7[00-FF]
0x103 x8 n=5002   03 00 02 00 00 00 05 04  var: b0[00-7D] b2[00-02] b6[00-0F] b7[00-7F]
0x122 x8 n=5002   15 17 13 0B 00 00 07 1D  var: b0[05-25] b1[00-17] b2[00-4A] b3[09-FF] b4[00-14] b5[00-FF] b6[00-0F] b7[00-FF]
0x013 x8 n=911    FF 07 09 00 00 00 00 00  var: none (all bytes constant)
```

## C. Time series — idle, rev to 6554, back to idle, shutdown

Decimated to roughly every 0.6 s. RPM decoded from 0x102 b0/b1 big-endian /4.
Gauge read 1400-1500 at idle and ~6500 at the top of the rev.

```
     t      RPM   raw   0x122 b0 b1 b2 b3 b4 b5   110.b3  320.b4  342.b4
   0.0s    1334  14D8   16 17 11 10 02 1C          16      F0      8C
   0.6s    1452  16B0   14 17 0F 0E 01 60          14      F0      8C
   1.2s    1480  1722   13 17 0E 0D 00 F3          14      F0      8D
   1.8s    1508  1792   14 17 0E 0E 01 1D          14      F0      8C
   2.4s    1485  1736   13 17 0E 0D 01 0F          13      F0      8C
   3.0s    1458  16C8   14 17 0E 0E 01 30          14      F0      8C
   3.6s    1459  16CD   14 17 0E 0E 01 03          14      F0      8C
   4.2s    1373  1576   14 17 0F 0E 01 5D          14      F0      8C
   4.8s    1491  174E   14 17 0E 0E 01 11          14      F0      8D
   5.4s    1541  1814   14 17 0E 0E 01 18          14      F0      8D
   6.0s    1441  1686   14 17 0E 0E 01 1F          14      F0      8D
   6.7s    1435  166F   14 17 0E 0E 01 4E          14      F0      8C
   7.3s    2049  2007   16 15 10 10 01 96          16      F0      8D
   7.9s    2319  243E   14 11 0F 0F 01 1B          14      F0      8F
   8.5s    2581  2854   14 0B 11 11 01 91          14      F0      8D
   9.1s    3065  2FE4   12 03 10 11 01 2A          12      F0      8F
   9.7s    3178  31AA   12 02 10 11 01 06          12      F0      8D
  10.3s    3817  3BA7   12 00 13 12 01 81          12      F0      90
  10.9s    4003  3E8C   13 00 13 13 01 8C          13      F0      8F
  11.5s    4550  4718   14 00 13 14 01 1B          14      F0      90
  12.1s    5015  4E5E   16 00 14 16 00 E7          16      F0      8F
  12.7s    5309  52F5   17 00 16 17 01 85          17      F0      8F
  13.3s    5645  5836   18 00 17 18 01 ED          18      F0      90
  13.9s    6263  61DE   18 00 18 18 01 EC          18      F0      8F
  14.5s    6554  6668   18 00 19 18 01 ED          18      F0      8D   <- peak
  15.1s    4670  48F8   11 00 10 11 00 00          11      F0      8F   <- throttle shut
  15.7s    2862  2CB8   0B 07 0A 09 00 00          0B      F0      8D
  16.3s    1588  18D1   13 17 0D 0D 01 23          13      F0      8D
  16.9s    1574  189A   14 17 0E 0E 01 F1          14      F0      8C
  17.5s    1297  1446   14 17 10 0E 01 AA          14      F0      8C
  ...
  22.4s       0  0000   13 17 12 0B 00 00          13      F0      88   <- engine off
```

After shutdown, everything freezes except **0x342 b4**, which drifts down
steadily over the following forty seconds:

```
  22.4s 88   26.6s 86   33.8s 85   44.1s 84   56.1s 83
```

That decay with the engine stopped is the reason 0x342 b4 is read as a
temperature rather than as another RPM-derived signal.

## D. Correlations against confirmed RPM

Over the 30 engine-running samples above.

```
  0x122 b1   r = -0.920   0x17 idle -> 0x00 wide open
  0x122 b3   r = +0.893
  0x122 b2   r = +0.865
  0x342 b4   r = +0.752
  0x122 b0   r = +0.341
  0x110 b3   r = +0.331
  0x122 b4   r = -0.219
```
