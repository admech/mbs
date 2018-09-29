within MbsLite;

model FooBar
  Real x(start = 1);
equation
  der(x) = 1;
end FooBar;
