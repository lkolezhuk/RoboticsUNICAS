function [ jacobian ] = GetJacobian( DH )
%Computes the Jacobian with given DH matrix.
    [DH_size,~] = size(DH);
    [p_vectors,z_vectors] = GetPZVectors(DH);
    jacobian = zeros(6, DH_size);
    
    for i = 1:DH_size
        jacobian(1:3,i) = cross(z_vectors{i}, p_vectors{DH_size+1} - p_vectors{i});
        jacobian(4:6,i) = z_vectors{i};
    end
        
end

