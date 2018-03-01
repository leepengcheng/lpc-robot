# #coding:utf-8
# function include(absolutePathAndFile)
#     if not __notFirst__ then
#         __notFirst__=true
#         __scriptCodeToRun__=assert(loadfile(absolutePathAndFile))
#     end
#     if __scriptCodeToRun__ then
#         __scriptCodeToRun__()
#     end
# end
# include('E:/Work/VREP/motionplanningDemo.lua')
import os
import zipfile
import re
import itertools
from lxml import etree
import json
from platform import system
from getpass import getuser


def parseApiDocument(apiHtmlFile):
    '''解析帮助文档中的API说明'''
    ApiDict={}
    with open(apiHtmlFile,"rt") as f:
        html=etree.HTML(f.read().decode("utf-8"))
        ApiNodes=html.xpath("//h3[@class='subsectionBar']")
        apiParaDict={}
        for node in ApiNodes:
            apiName=node.find("a").attrib['name'] #api名称
            #废弃的API
            if "DEPRECATED" in node.xpath("string(.)"):
                continue
            apiTable=node.getnext()
            description=apiTable.xpath(".//td[@class='apiTableRightDescr']")[0].xpath("string(.)")
            params_str=[]
            paramsNode=apiTable.xpath(".//td[@class='remApiTableRightCParam']/div/strong")
            for p in paramsNode:
                # paramName=p.text
                parent=p.getparent()
                paramDes=parent.xpath("string(.)") #获得参数说明
                params_str.append(paramDes)

            retVal_str=[]
            retValsNode=apiTable.xpath(".//td[@class='remApiTableRightCRet']/div/strong")
            for ret in retValsNode:
                # retName=ret.text
                parent=p.getparent()
                retDes=parent.xpath("string(.)") #获得返回值说明
                retVal_str.append(retDes)
            ApiDict[apiName]=[description,"\n".join(params_str),"\n".join(retVal_str)]
    return ApiDict





UPDATE_EXTENSION=False  #是否更新本地Lua插件


SYS=system()
USER=getuser()
# VREP_ROOT=r"C:\Program Files\V-REP3\V-REP_PRO_EDU"
VREP_ROOT=os.getenv('VREP_ROOT')
if not os.path.exists(VREP_ROOT):
    raise NameError("VREP_ROOT is not exist")



remoteApiHelpfiles=os.path.join(VREP_ROOT,"helpFiles/en/remoteApiFunctionsLua.htm")
apiDict=parseApiDocument(remoteApiHelpfiles)

if SYS=="Linux":
    luaExtension="/home/%s/.vscode/extensions/keyring.lua-0.0.9/snippets/snippets.json"%USER
else:
    luaExtension = "C:/Users/%s/.vscode/extensions/keyring.lua-0.0.9/snippets/snippets.json"%USER
if not os.path.exists(luaExtension):
    raise NameError("luaExtension is not exist")




luaExtensionStream=open(luaExtension,"rt")
data = luaExtensionStream.read()
snippetsDict = eval(data)

zip = zipfile.ZipFile(os.path.join(VREP_ROOT,"v-rep_notepad++.zip"))
# print(zip.namelist())
# 常亮
vrep_const = zip.read('keywords/v_repConstants.txt')
vrep_const_str = vrep_const.split()

# 函数
vrep_funcs = zip.read('keywords/v_repFunctions.txt')
# print(vrep_funcs)


vrep_func_xml = zip.read('auto-completion/V-REP.xml')
vrep_func_xml = re.sub(r'".*<(.*)>"', r'"\1"', vrep_func_xml)
xml = etree.fromstring(vrep_func_xml)
keyNodes = xml.xpath("//KeyWord")
for node in keyNodes:
    funcName = node.attrib["name"]  # 函数名
    prefix = funcName  # 前缀
    description = "Constants"  # 描述
    body = funcName  # 函数体
    # 如果是函数
    if "func" in node.keys():
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
    snippet = '''{
        "body": "%s",
        "description": "%s",
        "prefix": "%s",
        "scope": "source.lua"
    }''' % (body, description, prefix)
    snippetsDict[funcName] = eval(snippet)
zip.close()
luaExtensionStream.close()

settingfile=luaExtension
if not UPDATE_EXTENSION:
   settingfile='./snippets.json'
with open(settingfile, 'w') as f:
    json.dump(snippetsDict, f, indent=4)
