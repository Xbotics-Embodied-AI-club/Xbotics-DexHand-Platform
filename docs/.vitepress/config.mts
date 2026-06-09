import { defineConfig } from 'vitepress'
// https://vitepress.dev/reference/site-config

// 1. 获取环境变量并判断
// 如果环境变量 EDGEONE 等于 '1'，说明在 EdgeOne 环境，使用根路径 '/'
// 否则默认是 GitHub Pages 环境，使用仓库子路径 '/Xbotics_dexhand_repo_demo/'
const isEdgeOne = process.env.EDGEONE === '1'
const baseConfig = isEdgeOne ? '/' : '/Xbotics-DexHand-Platform/'

export default defineConfig({
  lang: 'zh-CN',
  title: "Xbotics 灵巧手知识库",
  description: "AI前沿知识开源教程",
  base: baseConfig,
  markdown: {
    math: true
  },
  ignoreDeadLinks: true,
  themeConfig: {
    // https://vitepress.dev/reference/default-theme-config
    logo: '/logo.jpg',
    nav: [
      { text: 'PDF版本下载', link: 'https://github.com/xbotics-embodied-ai-club.github.io/Xbotics-DexHand-Platform/releases' },
    ],
    search: {
      provider: 'local',
      options: {
        translations: {
          button: {
            buttonText: '搜索文档',
            buttonAriaLabel: '搜索文档'
          },
          modal: {
            noResultsText: '无法找到相关结果',
            resetButtonTitle: '清除查询条件',
            footer: {
              selectText: '选择',
              navigateText: '切换'
            }
          }
        }
      }
    },
    sidebar: [
      {
        text: '第1章：灵心巧手实训营',
        link: '/chapter1/',
        items: [
          { text: '第一个月：基础入门（第 1–4 周）', link: '/chapter1/1_1/' },
          { text: '第二个月：仿真进阶（第 5–8 周）', link: '/chapter1/1_2/' },
          { text: '第三个月：线下实操（第 9–12 周）', link: '/chapter1/1_3/' }
        ]
      },
      { text: '第2章：开源硬件手（五指 / 人形 & 可复现）', link: '/chapter2/' },
      { text: '第3章：低成本 / 三指平台（研究基线 & 任务平台）', link: '/chapter3/' },
      { text: '第4章：仿真环境 & 学术基准（手部）', link: '/chapter4/' },
      { text: '第5章：数据集（Dexterous Grasp / In-hand / 演示）', link: '/chapter5/' },
      { text: '第6章：算法 / 策略与遥操作（附代码）', link: '/chapter6/' },
      { text: '第7章：触觉硬件 & 软件栈（适合灵巧操作）', link: '/chapter7/' },
      { text: '第8章：如何使用这个知识库', link: '/chapter8/' },
    ],

    socialLinks: [
      { icon: 'github', link: 'https://github.com/xbotics-embodied-ai-club.github.io/Xbotics-DexHand-Platform' }
    ],

    editLink: {
      pattern: 'https://github.com/xbotics-embodied-ai-club.github.io/Xbotics-DexHand-Platform/blob/main/docs/:path'
    },

    footer: {
      message: 'Xbotics 具身智能实验室',
    //  message: '<a href="https://beian.miit.gov.cn/" target="_blank">京ICP备2026002630号-1</a> | <a href="https://beian.mps.gov.cn/#/query/webSearch?code=11010602202215" rel="noreferrer" target="_blank">京公网安备11010602202215号</a>',
    //  copyright: '本作品采用 <a href="http://creativecommons.org/licenses/by-nc-sa/4.0/" target="_blank">知识共享署名-非商业性使用-相同方式共享 4.0 国际许可协议（CC BY-NC-SA 4.0）</a> 进行许可'
    }
  }
})
