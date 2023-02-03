import subprocess
import os


subprocess.call("sed")


rcd_dir=os.listdir("../")


for file in rcd_dir:
    if file.startswith("Bus") | file.startswith("DDR5"):
        print(file)
        # key_word = "\'/TestBed/"
        # cat_1 = "cat test_template_head.py > test_" + file
        # sed_cmd = "sed -n " +key_word+ ",$p\' " + "../"+file +" >> test_"+file
        # cat_2 = "cat test_template_body.py >> test_" + file    
        # os.system(cat_1)
        # os.system(sed_cmd)
        # os.system(cat_2)
