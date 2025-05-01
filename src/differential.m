function d = differential(vector1,vector2)
  arguments
    vector1 (:,1) double
    vector2 double = []
  end
  
if ~isempty(vector2)
  d = diff(vector1) ./ diff(vector2)';
else
    d = diff(vector1); 
end
end
    