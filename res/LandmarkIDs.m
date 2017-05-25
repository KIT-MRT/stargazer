clc;clear;
ID = 24832%hex2dec('884')
marke = zeros(4,4);
marke([1,4,4],[1,1,4])=8;
for i=1:4
    col = mod(ID,16^i) / 16^(i-1);
    col_bin = dec2bin(col,4);
    for j=1:4
        marke(5-j,i) = str2num(col_bin(j));
    end
    ID = ID-col;
end
marke
%%
ID = hex2dec('400')
marke = zeros(4,4);
marke([1,4,4],[1,1,4])=8;
for i=1:4
    col = floor(mod(ID,16^i) / 16^(i-1));
    ID = ID-col;
    for j=1:4
        if mod(col,2) ~= 0
            marke(j,i) = 1;
        end
        col = floor(col/2);
    end
end
marke
