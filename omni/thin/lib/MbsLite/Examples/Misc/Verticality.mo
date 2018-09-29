within MbsLite.Examples.Misc;

model Verticality

  parameter String name = "NOT INITIALIZED";

  KinematicPort  InPort;
  WrenchPort     OutPort;

  Real[3]        torque;

initial algorithm
  AssertInitializedS(name, name, "name");

equation

  OutPort.P = InPort.r;
  OutPort.F = zeros(3);
  OutPort.M = torque;

end Verticality;
