#include "user/dfs.h"
#include <string.h>  // 使用 strcpy
// 注意：此模块仅使用 C 标准库，不包含 iostream 等 C++ 库

#define MAX_PATH_LEN 20  // 假设最大路径长度

// 内部函数：将方向数字映射为对应字符
static char mapDir(int d) {
    switch(d) {
        case 1: return 'U';  // 上
        case 2: return 'L';  // 左
        case 3: return 'D';  // 下
        case 4: return 'R';  // 右
        default: return '?';
    }
}

// 内部递归辅助函数
// 参数说明：
//   current    当前节点编号
//   goal       目标节点编号
//   visited    访问标记数组
//   tempPath   用于存储路径字符（逆序存储）
//   index      指向 tempPath 当前存放位置的索引（传地址，便于递归累加）
// 返回：找到路径则返回 1，否则返回 0
static int dfsHelper(int current, int goal, int *visited, char *tempPath, int *index) {
    if (current == goal) {
        return 1;
    }
    visited[current] = 1;
    for (int i = 0; i < NODE_COUNT; i++) {
        if (edge[current][i] != 0 && !visited[i]) {
            if (dfsHelper(i, goal, visited, tempPath, index)) {
                // 记录从当前节点到 i 的边所对应的方向字符（后加入数组中，最后需逆序）
                tempPath[(*index)++] = mapDir(edge[current][i]);
                return 1;
            }
        }
    }
    visited[current] = 0;  // 回溯时重置标记
    return 0;
}

// 对外公开的 dfs() 函数，调用后会将路径存入 result 数组中
void dfs(int start, int goal, char *result) {
    int visited[NODE_COUNT] = {0};
    char tempPath[MAX_PATH_LEN] = {0};
    int index = 0;
    
    if (dfsHelper(start, goal, visited, tempPath, &index)) {
        // 逆转 tempPath 数组，使路径从起点到终点顺序排列
        for (int i = 0; i < index / 2; i++) {
            char temp = tempPath[i];
            tempPath[i] = tempPath[index - i - 1];
            tempPath[index - i - 1] = temp;
        }
        tempPath[index] = '\0';
        // 将最终路径复制到 result 中
        strcpy(result, tempPath);
    } else {
        // 无路径则置空字符串
        result[0] = '\0';
    }
}
