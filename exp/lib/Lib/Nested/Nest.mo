within Lib.Nested;

model Nest
  parameter Real b = 2;
  Real y(start = 1);
equation
  der(y) = y - b;
end Nest;
