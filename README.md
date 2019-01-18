# Pipelined-MIPS-Design
2018 Digital System Design Final project

## Introduction
This repo is our implementation of pipelined MIPS (Microprocessor without Interlocked Pipeline Stages).
All the specificatoin is under the Specification/ folder. 

## Usage
The codes needed is under the Specificatoin/ and Src/ folder.

### Base line
RTL level:
```
ncverilog Final_tb.v CHIP.v slow_memory.v +define+hasHazard +access+r
```
Gate level:
```
ncverilog Final_tb.v CHIP_syn.v slow_memory.v tsmc13.v +define+hasHazard +define+SDF +access+r
```

### Extension - Branch Prediction
RTL level:
* no hazard
```
ncverilog Final_tb.v CHIP.v slow_memory.v +define+BrPred +access+r
```
* has hazard
```
ncverilog Final_tb.v CHIP.v slow_memory.v +define+hasHazard +access+r
```

Gate level:
* no hazard
```
ncverilog Final_tb.v CHIP_syn.v slow_memory.v tsmc13.v +define+BrPred +define+SDF +access+r
```
* has hazard
```
ncverilog Final_tb.v CHIP_syn.v slow_memory.v tsmc13.v +define+hasHazard +define+SDF +access+r
```

### Extension - Two level Cache
RTL level:
```
ncverilog Final_tb.v CHIP.v slow_memory.v +define+L2Cache +access+r
```

Gate level:
```
ncverilog Final_tb.v CHIP_syn.v slow_memory.v tsmc13.v +define+L2Cache +define+SDF +access+r
```
### Extension - Multiplication & Devisoin
RTL level:
```
ncverilog Final_tb.v CHIP.v slow_memory.v +define+MultDiv +access+r
```

Gate level:
```
ncverilog Final_tb.v CHIP_syn.v slow_memory.v tsmc13.v +define+MultDiv +define+SDF +access+r
```
