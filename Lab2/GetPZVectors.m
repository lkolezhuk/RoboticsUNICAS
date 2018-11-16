function [ p_vectors, z_vectors ] = GetPZVectors( DH )
% Gets p-vectors and z-vectors for computing the Jacobian
    [DH_size,~] = size(DH);

    aframes = cell(DH_size);
    p_vectors = cell(DH_size);
    z_vectors = cell(DH_size);

    aframes{1} = NextFrameTransform(DH, 1);
    transformMatrix = aframes{1};
    p_vectors{1} = [0; 0; 0];
    p_vectors{2} = transformMatrix(1:3, 4);

    z_vectors{1} = [0; 0; 1];
    z_vectors{2} = transformMatrix(1:3, 3);
    disp(transformMatrix);
    
    for i = 2:DH_size
       aframes{i} = NextFrameTransform(DH, i);
       
       transformMatrix = transformMatrix * aframes{i};
       disp(transformMatrix);
       p_vectors{i+1} = transformMatrix(1:3, 4);
       z_vectors{i+1} = transformMatrix(1:3, 3);
%        disp(z_vectors{i});
    end
    
end

