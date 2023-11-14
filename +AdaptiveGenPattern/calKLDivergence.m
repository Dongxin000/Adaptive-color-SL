function [total] = calKLDivergence(w1, w2)
%%  Calculate Kullback-Leibler Divergence

%%
res1 = zeros([1,length(w1)]);
for i =1:length(w1)
    P =  w1(i);
    Q = w2(i);

    if(P == 0 && Q ~= 0)
        res1(i) = 0;
    elseif(P ~= 0 && Q == 0)
        res1(i) = P;
    elseif(P == 0 && Q == 0)
        res1(i) = 0;
    else
        res1(i) = P*log(P./Q);
    end
end

res2 = zeros([1,length(w2)]);
for i =1:length(w2)
    P = w2(i);
    Q = w1(i);
    
    if(P == 0 && Q ~= 0)
        res2(i) = 0;
    elseif(P ~= 0 && Q == 0)
        res2(i) = P;
    elseif(P == 0 && Q == 0)
        res2(i) = 0;
    else
        res2(i) = P*log(P./Q);
    end
end

total = sum(res1+res2);
end

