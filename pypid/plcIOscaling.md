| Vendor              | AI Card / Module Example              | 4 mA Raw Counts | 20 mA Raw Counts | Notes |
|---------------------|---------------------------------------|-----------------|------------------|-------|
| Rockwell (Allen-Bradley) | 1769-IF4 / 1769-IF8                 | 6240           | 31200           | Common CompactLogix / MicroLogix |
| Rockwell (Allen-Bradley) | 1756-IF8 / 1756-IF16                | ~6553          | 32767           | ControlLogix (16-bit) |
| Rockwell (Allen-Bradley) | Older PLC-5 / SLC                    | ~6208          | ~31208          | Legacy modules |
| Schneider (Modicon) | TM3AI4 / BMX AMI 0410               | ~6000          | ~31200          | Varies by extension module |
| Schneider (Modicon) | M221 / M241 Analog Inputs           | ~4000-6000     | ~20000-32767    | Check specific module |
| Siemens             | SM 1231 AI (S7-1200)                | 5530           | 27648           | Most common |
| Siemens             | S7-300 / S7-400 Analog Modules      | 0 or 5530      | 27648           | Depends on 0-20 / 4-20 config |
| Siemens (TI)        | TI 505 Analog Input                 | 6400           | 32000           | Legacy |
| Omron               | NX / CJ Series Analog Input         | 0              | 32767           | Often full 16-bit (check manual) |
| Mitsubishi          | FX5 / Q Series Analog Modules       | Varies         | Varies          | Typically 0-4000 or 0-32767 |
| Beckhoff            | EL3xxx Series (EtherCAT)            | 0              | 27648           | Similar to Siemens |
| ABB                 | AC500 Analog Inputs                 | 0              | 27648           | Common scaling |