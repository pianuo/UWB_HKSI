#!/usr/bin/env python3
"""
使用GitHub API上传项目到GitHub仓库
需要：GitHub用户名、仓库名、Personal Access Token
"""

import os
import base64
import json
import requests
from pathlib import Path
from typing import List, Tuple

# 需要用户提供的信息（如果为空，程序会提示输入）
GITHUB_USERNAME = ""  # GitHub用户名（留空会提示输入）
REPO_NAME = "UWB_HKSI"  # 仓库名称
GITHUB_TOKEN = ""  # Personal Access Token（留空会提示输入，不要提交到仓库）

# 需要忽略的文件和目录
IGNORE_PATTERNS = {
    "__pycache__", ".pyc", ".pyo", ".pyd", ".git",
    "venv", "env", ".venv", "ENV",
    ".vscode", ".idea", "*.swp", "*.swo",
    ".DS_Store", "Thumbs.db", "desktop.ini",
    "history", "*.jsonl", "run_output.txt",
    "*.log", "*.tmp", "*.bak"
}

def should_ignore(file_path: Path) -> bool:
    """检查文件是否应该被忽略"""
    path_str = str(file_path)
    for pattern in IGNORE_PATTERNS:
        if pattern in path_str:
            return True
    return False

def get_all_files(root_dir: Path) -> List[Path]:
    """获取所有需要上传的文件"""
    files = []
    for item in root_dir.rglob("*"):
        if item.is_file() and not should_ignore(item):
            files.append(item)
    return files

def read_file_content(file_path: Path) -> Tuple[str, str]:
    """读取文件内容并编码为base64"""
    try:
        with open(file_path, "rb") as f:
            content = f.read()
            # 检查是否是二进制文件
            try:
                text_content = content.decode("utf-8")
                encoded = base64.b64encode(content).decode("utf-8")
                return encoded, "base64"
            except UnicodeDecodeError:
                # 二进制文件，跳过
                return None, None
    except Exception as e:
        print(f"Error reading {file_path}: {e}")
        return None, None

def create_or_update_file(repo_owner: str, repo_name: str, token: str, 
                         file_path: str, content: str, message: str = "Upload file"):
    """创建或更新GitHub仓库中的文件"""
    # URL编码路径（处理中文和特殊字符）
    from urllib.parse import quote
    encoded_path = quote(file_path, safe="/")
    api_url = f"https://api.github.com/repos/{repo_owner}/{repo_name}/contents/{encoded_path}"
    
    headers = {
        "Authorization": f"token {token}",
        "Accept": "application/vnd.github.v3+json"
    }
    
    # 检查文件是否存在
    try:
        response = requests.get(api_url, headers=headers, timeout=10)
        sha = None
        if response.status_code == 200:
            sha = response.json().get("sha")
    except Exception as e:
        print(f"\n检查文件时出错: {e}")
        sha = None
    
    data = {
        "message": message,
        "content": content,
        "encoding": "base64"
    }
    if sha:
        data["sha"] = sha
    
    try:
        response = requests.put(api_url, headers=headers, json=data, timeout=10)
        if response.status_code not in [200, 201]:
            error_msg = response.text[:300] if len(response.text) > 300 else response.text
            print(f"\n错误: {response.status_code} - {error_msg}")
        return response.status_code in [200, 201]
    except Exception as e:
        print(f"\n上传时出错: {e}")
        return False

def create_repo(repo_owner: str, repo_name: str, token: str, 
                description: str = "UWB & GNSS Start Line Tracker"):
    """创建GitHub仓库"""
    api_url = "https://api.github.com/user/repos"
    headers = {
        "Authorization": f"token {token}",
        "Accept": "application/vnd.github.v3+json"
    }
    
    data = {
        "name": repo_name,
        "description": description,
        "private": False,
        "auto_init": False
    }
    
    response = requests.post(api_url, headers=headers, json=data)
    if response.status_code == 201:
        print(f"[OK] 仓库 {repo_name} 创建成功！")
        return True
    elif response.status_code == 422:
        print(f"[INFO] 仓库 {repo_name} 已存在，继续上传文件...")
        return True
    else:
        print(f"[ERROR] 创建仓库失败: {response.status_code}")
        try:
            error_msg = response.json()
            print(f"错误详情: {error_msg}")
        except:
            print(f"错误详情: {response.text[:500]}")
        return False

def main():
    """主函数"""
    import sys
    import io
    
    # 设置UTF-8编码输出
    if sys.platform == 'win32':
        sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
        sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')
    
    username = GITHUB_USERNAME
    token = GITHUB_TOKEN
    
    if not username:
        username = input("请输入您的GitHub用户名: ").strip()
        if not username:
            print("[ERROR] 用户名不能为空")
            return
    
    if not token:
        print("\n需要GitHub Personal Access Token")
        print("获取方法：")
        print("1. 访问: https://github.com/settings/tokens")
        print("2. 点击 'Generate new token (classic)'")
        print("3. 勾选 'repo' 权限")
        print("4. 生成后复制token\n")
        token = input("请输入您的GitHub Token: ").strip()
        if not token:
            print("[ERROR] Token不能为空")
            return
    
    # 创建仓库
    print(f"\n[INFO] 创建仓库 {REPO_NAME}...")
    if not create_repo(username, REPO_NAME, token):
        return
    
    # 获取所有文件
    root_dir = Path(".")
    files = get_all_files(root_dir)
    
    print(f"[INFO] 找到 {len(files)} 个文件需要上传...\n")
    
    # 上传文件
    success_count = 0
    fail_count = 0
    
    for i, file_path in enumerate(files, 1):
        relative_path = file_path.relative_to(root_dir)
        content, encoding = read_file_content(file_path)
        
        if content is None:
            print(f"[{i}/{len(files)}] [SKIP] 跳过: {relative_path} (二进制文件)")
            continue
        
        # 转换路径分隔符为/
        github_path = str(relative_path).replace("\\", "/")
        
        print(f"[{i}/{len(files)}] [UPLOAD] 上传: {github_path}...", end=" ")
        
        if create_or_update_file(username, REPO_NAME, token, 
                                 github_path, content, f"Add {github_path}"):
            print("OK")
            success_count += 1
        else:
            print("FAILED")
            fail_count += 1
    
    print(f"\n[SUCCESS] 完成！成功: {success_count}, 失败: {fail_count}")
    print(f"[LINK] 仓库地址: https://github.com/{username}/{REPO_NAME}")

if __name__ == "__main__":
    main()

