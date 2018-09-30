within MbsLite.Util.Assert;

function AssertInitialized

  input String modelName;
  input Real[:] value;
  input String parameterName;

algorithm
  
  assert(max(value) < inf, modelName + ": " + parameterName + " is not initialized");

end AssertInitialized;
