function [newMask1, newMask2, newMask3, newMask4] = removeNodePixels(imMask1, imMask2, imMask3, imMask4 )
%% Remove nodes' pixels of masks.

%%
% remove mask1 nodes' pixels
newMask1 =  imMask1.*(~imMask2);
newMask1 = newMask1.*(~imMask3);
newMask1 = newMask1.*(~imMask4);

% remove mask2 nodes' pixels
newMask2 = imMask2.*(~imMask1);
newMask2 = newMask2.*(~imMask3);
newMask2 = newMask2.*(~imMask4);

% remove mask3 nodes' pixels
newMask3 =  imMask3.*(~imMask1);
newMask3 = newMask3.*(~imMask2);
newMask3 = newMask3.*(~imMask4);

% remove mask4 nodes' pixels
newMask4 = imMask4.*(~imMask1);
newMask4 = newMask4.*(~imMask2);
newMask4 = newMask4.*(~imMask3);
end

