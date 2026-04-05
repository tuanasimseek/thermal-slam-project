function feat = extractCNNFeature(patch, net)

    if isa(patch, 'uint8')
        patch = im2double(patch);
    end

    if size(patch,3) == 1
        patch = repmat(patch, [1 1 3]);
    end

    inputSize = net.Layers(1).InputSize(1:2);
    patch = imresize(patch, inputSize);

    feat = activations(net, patch, 'pool5', 'OutputAs', 'rows');
    feat = double(feat(:))';
end