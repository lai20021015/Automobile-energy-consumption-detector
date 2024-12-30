To create checkboxes in markdown, you can use the following syntax:
- [v]中文字體獨立出來寫
- 把不同功能的程式碼拆開出來兩個function

兩件事你看看能否先做：
1. 把控制點從「時間」改成「位置」
2. 把「單一速限」改成「可變的速限」(跟位置有關)

即是速限是 speed_lim(x) where x is the location。是一個array
單一速限是  speed_lim(x) = upperbound, a constant 

這樣會比較符合實際狀況，例如轉彎處需放慢

===
