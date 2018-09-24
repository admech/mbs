within MbsLite;

function AssertInitializedA
  
  input Real[:,:] value;
  input String name;

algorithm
  
  assert(max(value) < inf, name + " is not initialized");

end AssertInitializedA;
