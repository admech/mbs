within MbsLite;

model Base
  extends BaseBody;

equation

  OutPort.r = { 0, 0, 0 };
  OutPort.v = { 0, 0, 0 };
  OutPort.a = { 0, 0, 0 };
  OutPort.T =
    [ 1, 0, 0
    ; 0, 1, 0
    ; 0, 0, 1
    ];
  OutPort.omega   = { 0, 0, 0 };
  OutPort.epsilon = { 0, 0, 0 };

end Base;
