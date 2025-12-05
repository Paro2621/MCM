function v = vex(S)
% Return the vector v that under skew() generates the skew symm matrix S 

    v = [S(3,2); S(1,3); S(2,1)];
end