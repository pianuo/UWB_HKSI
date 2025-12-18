#!/usr/bin/env python3
"""测试GitHub API连接"""

import requests

USERNAME = ""  # 请填写GitHub用户名
TOKEN = ""  # 请填写Personal Access Token

# 测试1: 获取用户信息
print("测试1: 获取用户信息...")
headers = {
    "Authorization": f"token {TOKEN}",
    "Accept": "application/vnd.github.v3+json"
}
response = requests.get("https://api.github.com/user", headers=headers)
print(f"状态码: {response.status_code}")
if response.status_code == 200:
    user_info = response.json()
    print(f"用户名: {user_info.get('login')}")
    print(f"显示名: {user_info.get('name')}")
else:
    print(f"错误: {response.text}")

# 测试2: 检查仓库是否存在
print(f"\n测试2: 检查仓库 {USERNAME}/UWB_HKSI...")
response = requests.get(f"https://api.github.com/repos/{USERNAME}/UWB_HKSI", headers=headers)
print(f"状态码: {response.status_code}")
if response.status_code == 200:
    repo_info = response.json()
    print(f"仓库存在: {repo_info.get('full_name')}")
elif response.status_code == 404:
    print("仓库不存在")
else:
    print(f"错误: {response.text}")

# 测试3: 尝试上传一个小文件
print(f"\n测试3: 尝试上传README.md...")
api_url = f"https://api.github.com/repos/{USERNAME}/UWB_HKSI/contents/README.md"
response = requests.get(api_url, headers=headers)
print(f"GET状态码: {response.status_code}")
if response.status_code == 200:
    file_info = response.json()
    print(f"文件已存在，SHA: {file_info.get('sha')[:20]}...")
else:
    print(f"文件不存在或无法访问")

