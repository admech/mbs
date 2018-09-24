within MbsLite;

function AssertInitializedI
  
  input Integer value;
  input String name;

algorithm
  
  assert(value > -Integer_inf, name + " is not initialized");

end AssertInitializedI;
