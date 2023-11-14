function [JSDivergence] = calJSDivergence(P,Q)
%% Calculate Jensen–Shannon divergence

%%
M = (P+Q)./2;
JSDivergence = sum( (P.*log(P./M))./2 + (Q.*log(Q./M))./2 );
end

