within MbsLite;

function AssertInitializedA
  
  input String modelName;
  input Real[:,:] value;
  input String parameterName;

algorithm
  
  assert(max(value) < inf, modelName + ": " + parameterName + " is not initialized");

end AssertInitializedA;
