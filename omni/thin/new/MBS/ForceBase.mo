within MBS;

model ForceBase
                extends Base;
  WrenchPort InPort;
equation
  InPort.P = zeros(3);
  InPort.F = zeros(3);
  InPort.M = zeros(3);
end ForceBase;
