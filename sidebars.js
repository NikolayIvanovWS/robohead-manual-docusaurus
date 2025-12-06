/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  // Название основной группы документации (может быть любым)
  tutorialSidebar: [
    // 1. intro.md (Главная страница)
    'intro', 

    // 2. 10-introduction (Введение)
    {
      type: 'category',
      label: 'Введение', // Заголовок в меню (из русского названия)
      collapsible: true, // Возможность сворачивать/разворачивать
      items: [
        'introduction/10-main-features',
        'introduction/20-equipment',
        'introduction/30-hardware-configuration',
        
        // Подраздел "Программное обеспечение"
        {
          type: 'category',
          label: 'Программное обеспечение',
          link: { type: 'doc', id: 'introduction/40-software' },
          items: [
            'introduction/41-robohead-controller',
            'introduction/42-display-driver',
            'introduction/43-neck-driver',
            'introduction/44-ears-driver',
            'introduction/45-speakers-driver',
            'introduction/46-respeaker-driver',
            'introduction/47-voice-recognizer-pocketsphinx',
            'introduction/48-sensor-driver',
            'introduction/49-usb-cam',
          ],
        },
        'introduction/50-useful-links',
      ],
    },

    // 3. getting-started (НАЧАЛО РАБОТЫ)
    {
      type: 'category',
      label: 'Начало работы',
      collapsible: true,
      items: [
        'getting-started/10-configuring-workplace-software',
        'getting-started/20-first-start',

        // Подраздел "Подключение к устройству"
        {
          type: 'category',
          label: 'Подключение к устройству',
          link: { type: 'doc', id: 'getting-started/30-connecting-to-device' },
          items: [
            'getting-started/31-connecting-via-ssh',
            'getting-started/32-connecting-via-vscode',
            'getting-started/33-connecting-via-sftp',
          ],
        },
        'getting-started/40-shutdown-device',
      ],
    },

    // 4. 30-setting-up-device (НАСТРОЙКА УСТРОЙСТВА)
    {
      type: 'category',
      label: 'Настройка устройства',
      collapsible: true,
      items: [
        'setting-up-device/10-setting-up-Wi-Fi-connection',
        'setting-up-device/20-connecting-without-pswd',

        // Подраздел "Изменение стандартных настроек устройства"
        {
          type: 'category',
          label: 'Изменение стандартных настроек устройства',
          link: { type: 'doc', id: 'setting-up-device/30-changing-device-settings' },
          items: [
            'setting-up-device/31-changing-network-name',
            'setting-up-device/32-changing-password',
            'setting-up-device/33-adjusting-volume',
            'setting-up-device/34-configuring-servos',
            'setting-up-device/35-microphone-backlight-control',
          ],
        },
        'setting-up-device/40-сhanging-standart-media-files',
        'setting-up-device/50-changing-keywords',
        'setting-up-device/60-updating-os-image',
      ],
    },

    // 5. 40-working-with-device (РАБОТА С УСТРОЙСТВОМ)
    {
      type: 'category',
      label: 'Работа с устройством',
      collapsible: true,
      items: [
        'working-with-device/10-changing-standart-action',
        'working-with-device/20-adding-new-action',
        'working-with-device/30-roboheadcontroller',
        'working-with-device/40-action_without_robohead_controller',
      ],
    },

    // 6. 50-interfacing_with_turtlebro
    {
      type: 'category',
      label: 'Взаимодействие с роботом TurtleBro',
      collapsible: true,
      items: [
        'interfacing_with_turtlebro/10-мounting-on-turtlebro',
        'interfacing_with_turtlebro/20-сonfiguring-communication',
        'interfacing_with_turtlebro/30-main-features',
        'interfacing_with_turtlebro/40-turtlebro_controller',
      ],
    },

    // 7. 60-interfacing_with_mors
    {
      type: 'category',
      label: 'Взаимодействие с робособакой МОРС',
      collapsible: true,
      items: [
        'interfacing_with_mors/10-мounting-on-mors',
        'interfacing_with_mors/20-сonfiguring-communication',
        'interfacing_with_mors/30-main-features',
        'interfacing_with_mors/40-mors_controller',
      ],
    },
  ],
};

module.exports = sidebars;