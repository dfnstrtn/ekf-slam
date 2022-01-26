let SessionLoad = 1
let s:so_save = &g:so | let s:siso_save = &g:siso | setg so=0 siso=0 | setl so=-1 siso=-1
let v:this_session=expand("<sfile>:p")
silent only
silent tabonly
cd F:/project_realm/school/project/dev/src/basic_slam
if expand('%') == '' && !&modified && line('$') <= 1 && getline(1) == ''
  let s:wipebuf = bufnr('%')
endif
set shortmess=aoO
badd +1 src/lib.rs
badd +48 ~/AppData/Local/nvim/init.vim
badd +12 plans
badd +22 term://F:/project_realm/school/project/dev/src/basic_slam//17136:C:/WINDOWS/system32/cmd.exe
badd +9 Cargo.toml
badd +2 ekf_slam/mod.rs
badd +4 term://F:/project_realm/school/project/dev/src/basic_slam//13020:C:/WINDOWS/system32/cmd.exe
badd +1 src/ekf_slam/mod.rs
badd +22 term://F:/project_realm/school/project/dev/src/basic_slam//9656:C:/WINDOWS/system32/cmd.exe
badd +1 src/ekf_slam/plans.txt
badd +0 term://F:/project_realm/school/project/dev/src/basic_slam//8508:C:/WINDOWS/system32/cmd.exe
argglobal
%argdel
edit src/lib.rs
let s:save_splitbelow = &splitbelow
let s:save_splitright = &splitright
set splitbelow splitright
wincmd _ | wincmd |
vsplit
1wincmd h
wincmd _ | wincmd |
split
1wincmd k
wincmd w
wincmd w
let &splitbelow = s:save_splitbelow
let &splitright = s:save_splitright
wincmd t
let s:save_winminheight = &winminheight
let s:save_winminwidth = &winminwidth
set winminheight=0
set winheight=1
set winminwidth=0
set winwidth=1
exe '1resize ' . ((&lines * 22 + 23) / 46)
exe 'vert 1resize ' . ((&columns * 87 + 87) / 174)
exe '2resize ' . ((&lines * 21 + 23) / 46)
exe 'vert 2resize ' . ((&columns * 87 + 87) / 174)
exe 'vert 3resize ' . ((&columns * 86 + 87) / 174)
argglobal
if bufexists("term://F:/project_realm/school/project/dev/src/basic_slam//8508:C:/WINDOWS/system32/cmd.exe") | buffer term://F:/project_realm/school/project/dev/src/basic_slam//8508:C:/WINDOWS/system32/cmd.exe | else | edit term://F:/project_realm/school/project/dev/src/basic_slam//8508:C:/WINDOWS/system32/cmd.exe | endif
if &buftype ==# 'terminal'
  silent file term://F:/project_realm/school/project/dev/src/basic_slam//8508:C:/WINDOWS/system32/cmd.exe
endif
balt src/lib.rs
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
let s:l = 43 - ((21 * winheight(0) + 11) / 22)
if s:l < 1 | let s:l = 1 | endif
keepjumps exe s:l
normal! zt
keepjumps 43
normal! 051|
wincmd w
argglobal
balt plans
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let &fdl = &fdl
let s:l = 1 - ((0 * winheight(0) + 10) / 21)
if s:l < 1 | let s:l = 1 | endif
keepjumps exe s:l
normal! zt
keepjumps 1
normal! 0
wincmd w
argglobal
if bufexists("src/ekf_slam/mod.rs") | buffer src/ekf_slam/mod.rs | else | edit src/ekf_slam/mod.rs | endif
if &buftype ==# 'terminal'
  silent file src/ekf_slam/mod.rs
endif
balt term://F:/project_realm/school/project/dev/src/basic_slam//13020:C:/WINDOWS/system32/cmd.exe
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let &fdl = &fdl
let s:l = 67 - ((35 * winheight(0) + 22) / 44)
if s:l < 1 | let s:l = 1 | endif
keepjumps exe s:l
normal! zt
keepjumps 67
normal! 018|
wincmd w
3wincmd w
exe '1resize ' . ((&lines * 22 + 23) / 46)
exe 'vert 1resize ' . ((&columns * 87 + 87) / 174)
exe '2resize ' . ((&lines * 21 + 23) / 46)
exe 'vert 2resize ' . ((&columns * 87 + 87) / 174)
exe 'vert 3resize ' . ((&columns * 86 + 87) / 174)
tabnext 1
if exists('s:wipebuf') && len(win_findbuf(s:wipebuf)) == 0&& getbufvar(s:wipebuf, '&buftype') isnot# 'terminal'
  silent exe 'bwipe ' . s:wipebuf
endif
unlet! s:wipebuf
set winheight=1 winwidth=20 shortmess=filnxtToOF
let &winminheight = s:save_winminheight
let &winminwidth = s:save_winminwidth
let s:sx = expand("<sfile>:p:r")."x.vim"
if filereadable(s:sx)
  exe "source " . fnameescape(s:sx)
endif
let &g:so = s:so_save | let &g:siso = s:siso_save
set hlsearch
nohlsearch
doautoall SessionLoadPost
unlet SessionLoad
" vim: set ft=vim :
