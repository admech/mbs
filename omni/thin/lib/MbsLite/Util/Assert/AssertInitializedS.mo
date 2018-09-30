within MbsLite.Util.Assert;

function AssertInitializedS

  input String modelName;
  input String value;
  input String parameterName;

algorithm
  
  assert(value <> "NOT INITIALIZED", modelName + ": " + parameterName + " is not initialized");

end AssertInitializedS;
