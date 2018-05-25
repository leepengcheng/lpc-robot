#coding:utf-8
import os
import zipfile
import re
import itertools
from lxml import etree
import json
from platform import system
from getpass import getuser
import glob

def parseApiDocument(apiHtmlFile,isRemoteAPi=True):
    '''解析帮助文档中的API说明'''
    ApiDict={}
    with open(apiHtmlFile,"rt") as f:
        html=etree.HTML(f.read().decode("utf-8"))
        ApiNodes=html.xpath("//h3[@class='subsectionBar']")
        for node in ApiNodes:
            apiName=node.find("a").attrib['name'] #api名称
            #废弃的API
            if "DEPRECATED" in node.xpath("string(.)"):
                continue
            apiTable=node.getnext()
            description=apiTable.xpath(".//td[@class='apiTableRightDescr']")[0].xpath("string(.)")
            params_str=[]
            if isRemoteAPi:
                paramsNode=apiTable.xpath(".//td[@class='remApiTableRightCParam']/div/strong")
            else:
                paramsNode=apiTable.xpath(".//td[@class='apiTableRightCParam']/div")
            for p in paramsNode:
                # paramName=p.text
                parent=p.getparent()
                paramDes=parent.xpath("string(.)") #获得参数说明
                params_str.append(paramDes)
            retVal_str=[]
            if isRemoteAPi:
                retValsNode=apiTable.xpath(".//td[@class='remApiTableRightCRet']/div/strong")
            else:
                retValsNode=apiTable.xpath(".//td[@class='apiTableRightCRet']/div")
                
            for ret in retValsNode:
                # retName=ret.text
                parent=ret.getparent()
                retDes=parent.xpath("string(.)") #获得返回值说明
                retVal_str.append(retDes)
            ApiDict[simToSimDot(apiName)]=[description,"\n".join(params_str),"\n".join(retVal_str)]
    return ApiDict



def simToSimDot(funcName):
    index=0
    nameList=list(funcName)
    for x in nameList:
        if x.isupper():
            break
        index=index+1
    nameList[index]=nameList[index].lower()
    nameList.insert(index,".")
    return "".join(nameList)




UPDATE_EXTENSION=True  #是否更新本地Lua插件
apiDict={}

SYS=system()
USER=getuser()
# VREP_ROOT=r"C:\Program Files\V-REP3\V-REP_PRO_EDU"
VREP_ROOT=os.getenv('VREP_ROOT')
if not os.path.exists(VREP_ROOT):
    raise EnvironmentError("VREP_ROOT is not exist")

remoteApiHelpfiles=os.path.join(VREP_ROOT,"helpfiles/en/remoteApiFunctionsLua.htm")
if  os.path.exists(remoteApiHelpfiles):
    apiDict.update(parseApiDocument(remoteApiHelpfiles))
else:
    raise EnvironmentError("NO VREP HELP FILE")

if SYS=="Linux":
    luaExtension="/home/%s/.vscode/extensions/keyring.lua-0.0.9/snippets/snippets.json"%USER
else:
    luaExtension = "C:/Users/%s/.vscode/extensions/keyring.lua-0.0.9/snippets/snippets.json"%USER
if not os.path.exists(luaExtension):
    raise NameError("luaExtension is not exist")




regularApiHelpfiles=glob.glob("%s/helpfiles/en/regularApi/*.htm"%VREP_ROOT)
for apiFile in regularApiHelpfiles:
    apiDict.update(parseApiDocument(apiFile,False))

luaExtensionStream=open(luaExtension,"rt")
data = luaExtensionStream.read()
snippetsDict = eval(data)




zip = zipfile.ZipFile(os.path.join(VREP_ROOT,"v-rep_notepad++.zip"))


# 函数
vrep_func_xml = zip.read('lua.xml')
vrep_func_xml = re.sub(r'".*<(.*)>"', r'"\1"', vrep_func_xml)  #未转义
vrep_func_xml = vrep_func_xml.replace("func=yes","func=\"yes") #部分func未闭合
xml = etree.fromstring(vrep_func_xml)
keyNodes = xml.xpath("//KeyWord")

for node in keyNodes:
    funcName = node.attrib["name"]  # 函数名
    isFunc=True if node.attrib["func"]=="yes" else False
    prefix = funcName  # 前缀
    description = "Constants"  # 描述
    body = funcName  # 函数体
    # 如果是函数
    if isFunc:
        overloadNode = node.xpath("Overload")[0]
        retVal = overloadNode.attrib["retVal"]  # 函数返回值
        paramNodes = overloadNode.getchildren()  # 函数参数
        params = []
        # 如果有参数
        if len(paramNodes):
            for paramNode in paramNodes:
                params.append(paramNode.attrib["name"])
        paramNames = [x.split()[-1] for x in params if x.strip() != ""]
        # paramNamesFlatten=tuple(itertools.chain.from_iterable(enumerate(paramNames)))
        paramsBody = ("${%s:%s}" % (x, y) for x, y in enumerate(paramNames))
        body = "%s(%s)" % (funcName, ",".join(paramsBody))  # 函数体
        description = "%s %s(%s)" % (retVal, funcName, ",".join(params))
        if apiDict.has_key(funcName):
            description="%s\n%s"%(description," ".join(apiDict[funcName]))
        description=description.strip().replace("\"","").replace("\n","").replace(" "," ")
    snippet = '''{
        "body": "%s",
        "description": "%s",
        "prefix": "%s",
        "scope": "source.lua"
    }''' % (body, description, prefix)
    if "sim.addBanner" in funcName:
        pass
    snippetsDict[funcName] = eval(snippet)

luaExtensionStream.close()
zip.close()
settingfile=luaExtension
if not UPDATE_EXTENSION:
   settingfile='./snippets.json'
with open(settingfile, 'w') as f:
    json.dump(snippetsDict, f, indent=4)


# with open("axaa.json","wt") as f:
#     f.write(snippet)