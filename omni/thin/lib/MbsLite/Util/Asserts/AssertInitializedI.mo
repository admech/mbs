within MbsLite.Util.Asserts;

function AssertInitializedI
  
  input String modelName;
  input Integer value;
  input String parameterName;

algorithm
  
  assert(value > -Integer_inf, modelName + ": " + parameterName + " is not initialized");

end AssertInitializedI;
