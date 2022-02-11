# FlowChart

```flow
st=>start: Start
op=>operation: Set Paramamters and Initialize
op1=>operation: TrunOn
tr=>operation: Try Again
op2=>operation: doProcessSimple
op3=>operation: TrunOff
op4=>operation: disconnecting
cond=>condition: success Yes or No?
cond1=>condition: success Yes or No?
cond2=>condition: success Yes or No?
cond3=>condition: LOOP Yes or No?
cond4=>condition: TryAgain Yes or No?
e=>end: End
en=>end: End

st(left)->op->cond
cond(yes)->op1->cond1
cond(no)->op3->op4->e
cond1(yes)->op2->cond3
cond3(yes)->op2
cond3(no, left)->op3->op4->e
cond1(no,right)->tr(bottom)->cond4
cond4(yes)->op3
cond4(no)->op3(right)->op4(right)->e
```

# sequenceDiagram

```mermaid
sequenceDiagram
note over UserProgram: Set Paramters
note over UserProgram: Initialize SDK
UserProgram->Command: Get LiDAR Information
Command-->UserProgram: Device connected and Devce Information recevied
note over UserProgram: TurnOn
UserProgram->Command: Start LiDAR 
Command-->UserProgram: LiDAR Started successfully
UserProgram->LaserScan: Get Laser Scan Data
LaserScan-->UserProgram:  Laser Scan Data Recvied
note over UserProgram: doProcessSimple
loop Laser Scan Data
  LaserScan->UserProgram: doProcessSimple
end
note over UserProgram: TurnOff
UserProgram->Command: TurnOff 
note over UserProgram: disconnecting
UserProgram->Command: disconnecting
```
