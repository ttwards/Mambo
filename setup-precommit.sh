#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Pre-commit 设置脚本

set -e

echo "===================================="
echo "Pre-commit Hook 安装脚本"
echo "===================================="
echo ""

# 检查 Python 是否安装
if ! command -v python3 &> /dev/null; then
    echo "❌ 错误: 未找到 python3，请先安装 Python 3"
    exit 1
fi

echo "✓ Python 已安装: $(python3 --version)"

# 检查 pip 是否安装
if ! command -v pip3 &> /dev/null; then
    echo "❌ 错误: 未找到 pip3，请先安装 pip"
    exit 1
fi

echo "✓ pip 已安装"

# 安装 pre-commit
echo ""
echo "正在安装 pre-commit..."
pip3 install --user pre-commit

# 验证安装
if ! command -v pre-commit &> /dev/null; then
    echo "⚠️  警告: pre-commit 未在 PATH 中，请将 ~/.local/bin 添加到 PATH"
    echo "   可以运行: export PATH=\"\$HOME/.local/bin:\$PATH\""
    export PATH="$HOME/.local/bin:$PATH"
fi

echo "✓ pre-commit 已安装: $(pre-commit --version)"

# 安装 git hooks
echo ""
echo "正在安装 git hooks..."
pre-commit install

echo ""
echo "✓ Git hooks 已安装"

# 可选：运行一次检查
echo ""
read -p "是否要对所有文件运行一次 pre-commit 检查？(y/N) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "正在运行 pre-commit 检查..."
    pre-commit run --all-files || true
fi

echo ""
echo "===================================="
echo "✅ Pre-commit 设置完成！"
echo "===================================="
echo ""
echo "使用说明："
echo "  - 每次提交时会自动运行代码检查"
echo "  - 手动运行所有检查: pre-commit run --all-files"
echo "  - 跳过检查提交: git commit --no-verify"
echo "  - 更新 hooks: pre-commit autoupdate"
echo ""
