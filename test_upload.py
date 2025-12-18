#!/usr/bin/env python3
"""测试上传单个文件"""

import base64
import requests

USERNAME = ""  # 请填写GitHub用户名
REPO_NAME = "UWB_HKSI"
TOKEN = ""  # 请填写Personal Access Token

# 读取README.md
with open("README.md", "rb") as f:
    content = f.read()
    encoded = base64.b64encode(content).decode("utf-8")

# 准备API请求
api_url = f"https://api.github.com/repos/{USERNAME}/{REPO_NAME}/contents/README.md"
headers = {
    "Authorization": f"token {TOKEN}",
    "Accept": "application/vnd.github.v3+json"
}

# 检查文件是否存在
print("检查文件是否存在...")
response = requests.get(api_url, headers=headers)
print(f"GET状态码: {response.status_code}")

sha = None
if response.status_code == 200:
    sha = response.json().get("sha")
    print(f"文件已存在，SHA: {sha[:20]}...")
elif response.status_code == 404:
    print("文件不存在，将创建新文件")
else:
    print(f"错误: {response.status_code}")
    print(response.text)

# 尝试上传
print("\n尝试上传文件...")
data = {
    "message": "Add README.md",
    "content": encoded,
    "encoding": "base64"
}
if sha:
    data["sha"] = sha

response = requests.put(api_url, headers=headers, json=data)
print(f"PUT状态码: {response.status_code}")
if response.status_code in [200, 201]:
    print("上传成功！")
else:
    print(f"上传失败:")
    print(response.text)

