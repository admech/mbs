within Lib;

model Test
  parameter Real a = 1;
  Real x(start = 1);
equation
  der(x) = -x + a;
end Test;
