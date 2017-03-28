netstat -ano | findstr "5037"

-----------------------------------------------------------------------------------------------
最近在编译Android kernel时，遇到error, forbidden warning, 导致编译中断，大大降低了debug效率。

关闭方法如下：在kernel/scripts/gcc-wrapper.py中注释掉interpret_warning(line)即可。

--- a/scripts/gcc-wrapper.py

+++ b/scripts/gcc-wrapper.py

@@ -78,7 +78,7 @@ def run_gcc():

        proc = subprocess.Popen(args, stderr=subprocess.PIPE)

        for line in proc.stderr:

             print line,

-            interpret_warning(line)

+            #interpret_warning(line)

 

        result = proc.wait()

    except OSError as e:
    
    --------------------------------------------------------------------------------
